#include <iostream>
#include <cstring>
#include <cstdint>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <wx/wx.h>

// ─── RC522 Register Map ──────────────────────────────────────────────────────
namespace RC522Reg {
    constexpr uint8_t CommandReg      = 0x01;
    constexpr uint8_t ComlEnReg       = 0x02;
    constexpr uint8_t ComIrqReg       = 0x04;
    constexpr uint8_t ErrorReg        = 0x06;
    constexpr uint8_t FIFODataReg     = 0x09;
    constexpr uint8_t FIFOLevelReg    = 0x0A;
    constexpr uint8_t CollReg         = 0x0E;
    constexpr uint8_t BitFramingReg   = 0x0D;
    constexpr uint8_t ModeReg         = 0x11;
    constexpr uint8_t TxControlReg    = 0x14;
    constexpr uint8_t TxASKReg        = 0x15;
    constexpr uint8_t CRCResultRegH   = 0x21;
    constexpr uint8_t CRCResultRegL   = 0x22;
    constexpr uint8_t TModeReg        = 0x2A;
    constexpr uint8_t TPrescalerReg   = 0x2B;
    constexpr uint8_t TReloadRegH     = 0x2C;
    constexpr uint8_t TReloadRegL     = 0x2D;
    constexpr uint8_t VersionReg      = 0x37;
}

// ─── RC522 Commands ──────────────────────────────────────────────────────────
namespace RC522Cmd {
    constexpr uint8_t Idle            = 0x00;
    constexpr uint8_t CalcCRC         = 0x03;
    constexpr uint8_t Transceive      = 0x0C;
    constexpr uint8_t SoftReset       = 0x0F;
}

// ─── PICC Commands ───────────────────────────────────────────────────────────
namespace PICCCmd {
    constexpr uint8_t REQA            = 0x26;
    constexpr uint8_t SEL_CL1         = 0x93;
    constexpr uint8_t SEL_CL2         = 0x95;
    constexpr uint8_t SEL_CL3         = 0x97;
}

// ─── Status Codes ────────────────────────────────────────────────────────────
enum class Status { OK, Error, Timeout, Collision };



// ─── RC522 Driver ────────────────────────────────────────────────────────────
class RC522 {


public:
    // gpioChip  : e.g. "/dev/gpiochip4" on RPi 5, "/dev/gpiochip0" on RPi 3/4
    // rstLine   : BCM GPIO number used for RST pin (e.g. 25)
    RC522(const char* spiDevice, const char* gpioChip, unsigned int rstLine)
        : spiFd_(-1), chip_(nullptr), rstRequest_(nullptr)

  
    {
        // ── Open SPI ────────────────────────────────────────────────────────
        spiFd_ = open(spiDevice, O_RDWR);
        if (spiFd_ < 0)
            throw std::runtime_error("Cannot open SPI device");

        uint8_t  mode  = SPI_MODE_0;
        uint8_t  bits  = 8;
        uint32_t speed = 1'000'000;

        ioctl(spiFd_, SPI_IOC_WR_MODE,          &mode);
        ioctl(spiFd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);

        // ── Open GPIO (libgpiod v2 API) ──────────────────────────────────────
        chip_ = gpiod_chip_open(gpioChip);
        if (!chip_)
            throw std::runtime_error("Cannot open GPIO chip");

        // Build a line settings object: output, initially LOW
        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings)
            throw std::runtime_error("Cannot create line settings");

        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        // Build a line config object and add our single line
        gpiod_line_config* lineCfg = gpiod_line_config_new();
        if (!lineCfg) {
            gpiod_line_settings_free(settings);
            throw std::runtime_error("Cannot create line config");
        }

        unsigned int offsets[1] = { rstLine };
        if (gpiod_line_config_add_line_settings(lineCfg, offsets, 1, settings) < 0) {
            gpiod_line_config_free(lineCfg);
            gpiod_line_settings_free(settings);
            throw std::runtime_error("Cannot add line settings to config");
        }

        // Build a request config (consumer label)
        gpiod_request_config* reqCfg = gpiod_request_config_new();
        if (!reqCfg) {
            gpiod_line_config_free(lineCfg);
            gpiod_line_settings_free(settings);
            throw std::runtime_error("Cannot create request config");
        }
        gpiod_request_config_set_consumer(reqCfg, "rc522-rst");

        // Request the line
        rstRequest_ = gpiod_chip_request_lines(chip_, reqCfg, lineCfg);

        // Free config objects (request keeps its own copy)
        gpiod_request_config_free(reqCfg);
        gpiod_line_config_free(lineCfg);
        gpiod_line_settings_free(settings);

        rstOffset_ = rstLine;

        if (!rstRequest_)
            throw std::runtime_error("Cannot request RST GPIO line");

        reset();
        init();
    }

    ~RC522() {
        if (rstRequest_) gpiod_line_request_release(rstRequest_);
        if (chip_)        gpiod_chip_close(chip_);
        if (spiFd_ >= 0)  close(spiFd_);
    }

    // Hardware reset via RST pin
    void reset() {
        setRst(GPIOD_LINE_VALUE_INACTIVE);   // pull LOW
        usleep(50'000);
        setRst(GPIOD_LINE_VALUE_ACTIVE);     // pull HIGH
        usleep(50'000);
        softReset();
    }

    // Returns firmware version (expect 0x91 or 0x92)
    uint8_t version() {
        return readReg(RC522Reg::VersionReg);
    }

    // Detect a card and read its UID (4-byte single-size UIDs)
    // Returns true and fills uid[]/uidLen on success
    bool readCard(uint8_t uid[], uint8_t& uidLen) {
        uint8_t atqa[2] = {};
        if (requestA(atqa) != Status::OK)
            return false;
        return (anticollision(uid, uidLen) == Status::OK);
    }

private:
    int                    spiFd_;
    gpiod_chip*            chip_;
    gpiod_line_request*    rstRequest_;
    unsigned int           rstOffset_;


    // ── RST pin helper ───────────────────────────────────────────────────────
    void setRst(gpiod_line_value val) {
        unsigned int offsets[1] = { rstOffset_ };
        gpiod_line_value values[1] = { val };
        gpiod_line_request_set_values_subset(rstRequest_, 1, offsets, values);
    }

    // ── SPI helpers ──────────────────────────────────────────────────────────
    void writeReg(uint8_t reg, uint8_t val) {
        uint8_t tx[2] = { static_cast<uint8_t>((reg << 1) & 0x7E), val };
        uint8_t rx[2] = {};
        spiTransfer(tx, rx, 2);
    }

    uint8_t readReg(uint8_t reg) {
        uint8_t tx[2] = { static_cast<uint8_t>(((reg << 1) & 0x7E) | 0x80), 0x00 };
        uint8_t rx[2] = {};
        spiTransfer(tx, rx, 2);
        return rx[1];
    }

    void setBitMask(uint8_t reg, uint8_t mask) {
        writeReg(reg, readReg(reg) | mask);
    }

    void clearBitMask(uint8_t reg, uint8_t mask) {
        writeReg(reg, readReg(reg) & ~mask);
    }

    void spiTransfer(uint8_t* tx, uint8_t* rx, size_t len) {
        spi_ioc_transfer tr{};
        tr.tx_buf        = reinterpret_cast<unsigned long>(tx);
        tr.rx_buf        = reinterpret_cast<unsigned long>(rx);
        tr.len           = static_cast<uint32_t>(len);
        tr.speed_hz      = 1'000'000;
        tr.bits_per_word = 8;
        ioctl(spiFd_, SPI_IOC_MESSAGE(1), &tr);
    }

    // ── RC522 initialisation ─────────────────────────────────────────────────
    void softReset() {
        writeReg(RC522Reg::CommandReg, RC522Cmd::SoftReset);
        usleep(50'000);
    }

    void init() {
        // Timer: auto-start, ~25 ms timeout
        writeReg(RC522Reg::TModeReg,      0x80);
        writeReg(RC522Reg::TPrescalerReg, 0xA9);
        writeReg(RC522Reg::TReloadRegH,   0x03);
        writeReg(RC522Reg::TReloadRegL,   0xE8);
        writeReg(RC522Reg::TxASKReg,      0x40); // 100% ASK
        writeReg(RC522Reg::ModeReg,       0x3D); // CRC seed 0x6363
        antennaOn();
 
    }

    void antennaOn() {
        uint8_t val = readReg(RC522Reg::TxControlReg);
        if ((val & 0x03) != 0x03)
            writeReg(RC522Reg::TxControlReg, val | 0x03);
    }

    // ── Transceive ───────────────────────────────────────────────────────────
    Status transceive(uint8_t* sendData, uint8_t sendLen,
                      uint8_t* recvData, uint8_t& recvLen,
                      uint8_t lastBits = 0)
    {
        writeReg(RC522Reg::ComlEnReg,    0x77);          // enable IRQs
        clearBitMask(RC522Reg::ComIrqReg, 0x80);         // clear IRQ bits
        writeReg(RC522Reg::CommandReg,   RC522Cmd::Idle); // stop any command
        setBitMask(RC522Reg::FIFOLevelReg, 0x80);         // flush FIFO

        for (uint8_t i = 0; i < sendLen; ++i)
            writeReg(RC522Reg::FIFODataReg, sendData[i]);

        writeReg(RC522Reg::BitFramingReg, lastBits);
        writeReg(RC522Reg::CommandReg, RC522Cmd::Transceive); // ← was truncated
        setBitMask(RC522Reg::BitFramingReg, 0x80);            // start transmission

        // Poll for completion (RxIRq=0x20, IdleIRq=0x10, TimerIRq=0x01)
        int     timeout = 2000;
        uint8_t irq     = 0;
        do {
            irq = readReg(RC522Reg::ComIrqReg);
            --timeout;
        } while (timeout > 0 && !(irq & 0x01) && !(irq & 0x30));

        clearBitMask(RC522Reg::BitFramingReg, 0x80);

        if (timeout == 0 || (irq & 0x01))
            return Status::Timeout;

        if (readReg(RC522Reg::ErrorReg) & 0x1B)
            return Status::Error;

        recvLen = readReg(RC522Reg::FIFOLevelReg);
        for (uint8_t i = 0; i < recvLen; ++i)
            recvData[i] = readReg(RC522Reg::FIFODataReg);

        return Status::OK;
    }

    // ── ISO 14443-3A: REQA (7-bit short frame) ───────────────────────────────
    Status requestA(uint8_t atqa[2]) {
        clearBitMask(RC522Reg::CollReg, 0x80);
        uint8_t cmd     = PICCCmd::REQA;
        uint8_t recvLen = 0;
        return transceive(&cmd, 1, atqa, recvLen, 7);
    }

    // ── ISO 14443-3A: Anticollision loop (CL1, 4-byte UID) ──────────────────
    Status anticollision(uint8_t uid[], uint8_t& uidLen) {
        writeReg(RC522Reg::BitFramingReg, 0x00);
        uint8_t cmd[2]  = { PICCCmd::SEL_CL1, 0x20 };
        uint8_t resp[5] = {};
        uint8_t respLen = 0;

        Status s = transceive(cmd, 2, resp, respLen);
        if (s != Status::OK) return s;

        if (respLen < 5)
            return Status::Error;

        // BCC: XOR of the 4 UID bytes must equal resp[4]
        if ((resp[0] ^ resp[1] ^ resp[2] ^ resp[3]) != resp[4])
            return Status::Error;

        uidLen = 4;
        memcpy(uid, resp, 4);
        return Status::OK;
    }

};

// ─── wxWidgets GUI ────────────────────────────────────────────────────────────
class RFIDFrame : public wxFrame {
public:
    wxTextCtrl* log_;

    RFIDFrame() : wxFrame(nullptr, wxID_ANY, "RFID Reader",
                          wxDefaultPosition, wxSize(400, 300))
    {
        log_ = new wxTextCtrl(this, wxID_ANY, "",
                              wxDefaultPosition, wxDefaultSize,
                              wxTE_MULTILINE | wxTE_READONLY);
    }

    void AppendLog(const wxString& msg) {
        log_->AppendText(msg + "\n");
    }
};

class RFIDApp : public wxApp {
public:
    RFIDFrame* frame_ = nullptr;

    bool OnInit() override {
        frame_ = new RFIDFrame();
        frame_->Show(true);
        return true;
    }
};

wxIMPLEMENT_APP_NO_MAIN(RFIDApp);  // ← lets us keep our own main()

// ─── Main ────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    // ── Boot wxWidgets ───────────────────────────────────────────────────────
    wxApp::SetInstance(new RFIDApp());
    wxEntryStart(argc, argv);
    wxTheApp->OnInit();

    RFIDFrame* frame = static_cast<RFIDApp*>(wxTheApp)->frame_;

    try {
        RC522 rfid("/dev/spidev0.0", "/dev/gpiochip0", 25);

        frame->AppendLog(wxString::Format("RC522 firmware: 0x%02X",
                                          rfid.version()));

        while (frame->IsShown()) {
            uint8_t uid[7] = {};
            uint8_t uidLen = 0;

            if (rfid.readCard(uid, uidLen)) {
                wxString s = "Card UID: ";
                for (uint8_t i = 0; i < uidLen; ++i) {
                    s += wxString::Format("%02X", uid[i]);
                    if (i + 1 < uidLen) s += ":";
                }
                frame->AppendLog(s);
                usleep(500'000);
            }

            wxTheApp->Yield();   // ← keeps the GUI responsive
            usleep(50'000);
        }

    } catch (const std::exception& e) {
        wxMessageBox(e.what(), "Fatal Error", wxICON_ERROR);
    }

    // ── Shut down wxWidgets ──────────────────────────────────────────────────
    wxEntryCleanup();
    return 0;
}
