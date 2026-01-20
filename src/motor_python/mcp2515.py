"""MCP2515 CAN Controller Driver for Waveshare RS485 CAN HAT.

This module provides low-level SPI communication with the MCP2515 CAN controller
used on the Waveshare RS485 CAN expansion board for Jetson.

Based on Waveshare sample code, adapted for motor control applications.
"""

import time
from dataclasses import dataclass

from loguru import logger

try:
    import spidev

    SPIDEV_AVAILABLE = True
except ImportError:
    SPIDEV_AVAILABLE = False
    logger.warning("spidev library not available. Install with: pip install spidev")


# MCP2515 Register Addresses
class MCP2515Registers:
    """MCP2515 register addresses."""

    # Configuration Registers
    CANSTAT = 0x0E
    CANCTRL = 0x0F
    BFPCTRL = 0x0C
    TEC = 0x1C
    REC = 0x1D
    CNF3 = 0x28
    CNF2 = 0x29
    CNF1 = 0x2A
    CANINTE = 0x2B
    CANINTF = 0x2C
    EFLG = 0x2D
    TXRTSCTRL = 0x0D

    # TX Buffer 0
    TXB0CTRL = 0x30
    TXB0SIDH = 0x31
    TXB0SIDL = 0x32
    TXB0EID8 = 0x33
    TXB0EID0 = 0x34
    TXB0DLC = 0x35
    TXB0D0 = 0x36

    # RX Buffer 0
    RXB0CTRL = 0x60
    RXB0SIDH = 0x61
    RXB0SIDL = 0x62
    RXB0EID8 = 0x63
    RXB0EID0 = 0x64
    RXB0DLC = 0x65
    RXB0D0 = 0x66

    # RX Buffer 1
    RXB1CTRL = 0x70
    RXB1SIDH = 0x71
    RXB1SIDL = 0x72
    RXB1EID8 = 0x73
    RXB1EID0 = 0x74
    RXB1DLC = 0x75
    RXB1D0 = 0x76

    # Receive Filters
    RXF0SIDH = 0x00
    RXF0SIDL = 0x01
    RXF0EID8 = 0x02
    RXF0EID0 = 0x03

    # Receive Masks
    RXM0SIDH = 0x20
    RXM0SIDL = 0x21
    RXM0EID8 = 0x22
    RXM0EID0 = 0x23


# MCP2515 SPI Commands
class MCP2515Commands:
    """MCP2515 SPI command bytes."""

    RESET = 0xC0
    READ = 0x03
    WRITE = 0x02
    RTS = 0x80
    RTS_TXB0 = 0x81
    RTS_TXB1 = 0x82
    RTS_TXB2 = 0x84
    READ_STATUS = 0xA0
    BIT_MODIFY = 0x05
    RX_STATUS = 0xB0
    READ_RX_BUFF = 0x90
    LOAD_TX = 0x40


# MCP2515 Mode Constants
class MCP2515Modes:
    """MCP2515 operation modes."""

    CONFIG = 0x80
    LISTEN = 0x60
    LOOPBACK = 0x40
    SLEEP = 0x20
    NORMAL = 0x00


# MCP2515 Baud Rate Constants (for 8MHz oscillator)
class MCP2515BaudRate:
    """MCP2515 baud rate settings for 8MHz crystal."""

    CAN_10Kbps = 0x31
    CAN_25Kbps = 0x13
    CAN_50Kbps = 0x09
    CAN_100Kbps = 0x04
    CAN_125Kbps = 0x03
    CAN_250Kbps = 0x01
    CAN_500Kbps = 0x00


# CNF2 Timing Constants
PHSEG1_3TQ = 0x10
PRSEG_1TQ = 0x00
PHSEG2_3TQ = 0x02

# Other Constants
CLKOUT_ENABLED = 0x04
DLC_8 = 0x08


@dataclass
class CANMessage:
    """CAN message structure."""

    can_id: int  # CAN identifier (11-bit standard or 29-bit extended)
    data: bytes  # Data bytes (0-8 bytes)
    dlc: int  # Data length code
    is_extended: bool = False  # Extended ID flag
    is_remote: bool = False  # Remote transmission request flag


class MCP2515:
    """MCP2515 CAN Controller Driver.

    This class provides direct SPI communication with the MCP2515 CAN controller
    used on the Waveshare RS485 CAN expansion board.
    """

    def __init__(
        self,
        spi_bus: int = 0,
        spi_device: int = 0,
        spi_speed: int = 10000000,
        baudrate: int = MCP2515BaudRate.CAN_500Kbps,
    ) -> None:
        """Initialize MCP2515 CAN controller.

        :param spi_bus: SPI bus number (default: 0).
        :param spi_device: SPI device/chip select (default: 0).
        :param spi_speed: SPI clock speed in Hz (default: 10MHz).
        :param baudrate: CAN baud rate setting (default: 500Kbps).
        :return: None
        """
        if not SPIDEV_AVAILABLE:
            raise ImportError(
                "spidev library is required for MCP2515 communication. "
                "Install with: pip install spidev"
            )

        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.spi_speed = spi_speed
        self.baudrate = baudrate
        self.spi: spidev.SpiDev | None = None
        self.connected = False
        self._connect()

    def _connect(self) -> None:
        """Establish SPI connection to MCP2515.

        :return: None
        """
        try:
            spi = spidev.SpiDev(self.spi_bus, self.spi_device)
            spi.max_speed_hz = self.spi_speed
            spi.mode = 0  # MCP2515 uses SPI mode 0
            self.spi = spi
            self.connected = True
            logger.info(
                f"Connected to MCP2515 on SPI{self.spi_bus}.{self.spi_device} "
                f"at {self.spi_speed / 1_000_000:.1f} MHz"
            )
        except Exception as e:
            logger.warning(f"Failed to connect to MCP2515: {e}")
            self.connected = False

    def _read_byte(self, addr: int) -> int:
        """Read a single byte from MCP2515 register.

        :param addr: Register address.
        :return: Register value.
        """
        if self.spi is None:
            return 0
        response = self.spi.xfer2([MCP2515Commands.READ, addr, 0x00])
        return response[2]

    def _write_byte(self, addr: int, data: int) -> None:
        """Write a single byte to MCP2515 register.

        :param addr: Register address.
        :param data: Data byte to write.
        :return: None
        """
        if self.spi is None:
            return
        self.spi.writebytes([MCP2515Commands.WRITE, addr, data])

    def _write_bytes(self, addr: int, data: list[int]) -> None:
        """Write multiple bytes to MCP2515 starting at address.

        :param addr: Starting register address.
        :param data: List of data bytes to write.
        :return: None
        """
        if self.spi is None:
            return
        self.spi.writebytes([MCP2515Commands.WRITE, addr] + data)

    def reset(self) -> None:
        """Reset MCP2515 to default state.

        :return: None
        """
        if self.spi is None:
            return
        self.spi.writebytes([MCP2515Commands.RESET])
        time.sleep(0.1)  # Wait for reset to complete
        logger.debug("MCP2515 reset complete")

    def init(self) -> bool:
        """Initialize MCP2515 for CAN communication.

        Configures the MCP2515 with:
        - Specified baud rate (default 500Kbps for 8MHz crystal)
        - Normal operation mode
        - RX buffer 0 enabled with interrupts

        :return: True if initialization successful, False otherwise.
        """
        if not self.connected or self.spi is None:
            logger.error("MCP2515 not connected")
            return False

        logger.info("Initializing MCP2515 CAN controller...")

        # Reset the chip
        self.reset()

        # Set baud rate (CNF1)
        # For 8MHz crystal, 500Kbps: CNF1=0x00
        self._write_byte(MCP2515Registers.CNF1, self.baudrate)

        # Set CNF2: SAM=0 (one sample), PHSEG1=3TQ, PRSEG=1TQ
        self._write_byte(MCP2515Registers.CNF2, 0x80 | PHSEG1_3TQ | PRSEG_1TQ)

        # Set CNF3: PHSEG2=3TQ
        self._write_byte(MCP2515Registers.CNF3, PHSEG2_3TQ)

        # Configure TX buffer 0 defaults
        self._write_byte(MCP2515Registers.TXB0SIDH, 0x00)
        self._write_byte(MCP2515Registers.TXB0SIDL, 0x00)
        self._write_byte(MCP2515Registers.TXB0DLC, DLC_8)

        # Configure RX buffer 0
        # RXM=11 (receive any message), BUKT=0 (no rollover)
        self._write_byte(MCP2515Registers.RXB0CTRL, 0x60)

        # Set filters and masks to accept all messages
        self._write_byte(MCP2515Registers.RXF0SIDH, 0x00)
        self._write_byte(MCP2515Registers.RXF0SIDL, 0x00)
        self._write_byte(MCP2515Registers.RXM0SIDH, 0x00)
        self._write_byte(MCP2515Registers.RXM0SIDL, 0x00)

        # Clear interrupt flags
        self._write_byte(MCP2515Registers.CANINTF, 0x00)

        # Enable RX0 interrupt
        self._write_byte(MCP2515Registers.CANINTE, 0x01)

        # Set normal operation mode with clock output enabled
        self._write_byte(MCP2515Registers.CANCTRL, MCP2515Modes.NORMAL | CLKOUT_ENABLED)

        # Verify we're in normal mode
        status = self._read_byte(MCP2515Registers.CANSTAT)
        if (status & 0xE0) != MCP2515Modes.NORMAL:
            logger.warning("MCP2515 not in normal mode, retrying...")
            self._write_byte(
                MCP2515Registers.CANCTRL, MCP2515Modes.NORMAL | CLKOUT_ENABLED
            )
            time.sleep(0.05)
            status = self._read_byte(MCP2515Registers.CANSTAT)

        if (status & 0xE0) == MCP2515Modes.NORMAL:
            logger.success("MCP2515 initialized successfully (500Kbps)")
            return True
        else:
            logger.error(f"MCP2515 initialization failed. Status: 0x{status:02X}")
            return False

    def send(self, can_id: int, data: list[int] | bytes, dlc: int = 8) -> bool:
        """Send a CAN message using TX buffer 0.

        :param can_id: 11-bit CAN identifier (0x000-0x7FF).
        :param data: Data bytes to send (up to 8 bytes).
        :param dlc: Data length code (0-8).
        :return: True if message sent successfully, False otherwise.
        """
        if not self.connected or self.spi is None:
            logger.debug("MCP2515 not connected - cannot send")
            return False

        # Convert data to list if bytes
        data_bytes: list[int] = list(data) if isinstance(data, bytes) else list(data)

        # Clamp DLC to 0-8
        dlc = max(0, min(8, dlc))

        # Check if TX buffer is available
        status = self._read_byte(MCP2515Commands.READ_STATUS)
        if status & 0x04:  # TXREQ bit set, buffer busy
            logger.debug("TX buffer busy, waiting...")
            time.sleep(0.01)
            self._write_byte(MCP2515Registers.TXB0CTRL, 0x00)  # Clear TXREQ
            # Wait for buffer to be free
            for _ in range(10):
                if not (self._read_byte(MCP2515Commands.READ_STATUS) & 0x04):
                    break
                time.sleep(0.001)

        # Set CAN ID (11-bit standard ID)
        # SIDH = ID[10:3], SIDL = ID[2:0] << 5
        self._write_byte(MCP2515Registers.TXB0SIDH, (can_id >> 3) & 0xFF)
        self._write_byte(MCP2515Registers.TXB0SIDL, (can_id & 0x07) << 5)

        # Extended ID bytes (not used for standard ID)
        self._write_byte(MCP2515Registers.TXB0EID8, 0x00)
        self._write_byte(MCP2515Registers.TXB0EID0, 0x00)

        # Set DLC
        self._write_byte(MCP2515Registers.TXB0DLC, dlc)

        # Write data bytes
        for i in range(min(dlc, len(data_bytes))):
            self._write_byte(MCP2515Registers.TXB0D0 + i, data_bytes[i])

        # Request to send
        self.spi.writebytes([MCP2515Commands.RTS_TXB0])

        logger.debug(
            f"TX: ID=0x{can_id:03X} DLC={dlc} "
            f"Data={' '.join(f'{b:02X}' for b in data_bytes[:dlc])}"
        )

        return True

    def send_extended(self, can_id: int, data: list[int] | bytes, dlc: int = 8) -> bool:
        """Send a CAN message with extended (29-bit) ID.

        :param can_id: 29-bit CAN identifier.
        :param data: Data bytes to send (up to 8 bytes).
        :param dlc: Data length code (0-8).
        :return: True if message sent successfully, False otherwise.
        """
        if not self.connected or self.spi is None:
            logger.debug("MCP2515 not connected - cannot send")
            return False

        # Convert data to list if bytes
        data_bytes: list[int] = list(data) if isinstance(data, bytes) else list(data)

        # Clamp DLC to 0-8
        dlc = max(0, min(8, dlc))

        # Check if TX buffer is available
        status = self._read_byte(MCP2515Commands.READ_STATUS)
        if status & 0x04:
            time.sleep(0.01)
            self._write_byte(MCP2515Registers.TXB0CTRL, 0x00)
            for _ in range(10):
                if not (self._read_byte(MCP2515Commands.READ_STATUS) & 0x04):
                    break
                time.sleep(0.001)

        # Set extended CAN ID (29-bit)
        # SIDH = ID[28:21]
        # SIDL = ID[20:18] << 5 | EXIDE | ID[17:16]
        # EID8 = ID[15:8]
        # EID0 = ID[7:0]
        sidh = (can_id >> 21) & 0xFF
        sidl = ((can_id >> 18) & 0x07) << 5 | 0x08 | ((can_id >> 16) & 0x03)
        eid8 = (can_id >> 8) & 0xFF
        eid0 = can_id & 0xFF

        self._write_byte(MCP2515Registers.TXB0SIDH, sidh)
        self._write_byte(MCP2515Registers.TXB0SIDL, sidl)
        self._write_byte(MCP2515Registers.TXB0EID8, eid8)
        self._write_byte(MCP2515Registers.TXB0EID0, eid0)

        # Set DLC
        self._write_byte(MCP2515Registers.TXB0DLC, dlc)

        # Write data bytes
        for i in range(min(dlc, len(data_bytes))):
            self._write_byte(MCP2515Registers.TXB0D0 + i, data_bytes[i])

        # Request to send
        self.spi.writebytes([MCP2515Commands.RTS_TXB0])

        logger.debug(
            f"TX (EXT): ID=0x{can_id:08X} DLC={dlc} "
            f"Data={' '.join(f'{b:02X}' for b in data_bytes[:dlc])}"
        )

        return True

    def receive(self, timeout: float = 0.1) -> CANMessage | None:
        """Receive a CAN message from RX buffer 0.

        :param timeout: Timeout in seconds to wait for message.
        :return: CANMessage object or None if no message received.
        """
        if not self.connected or self.spi is None:
            return None

        start_time = time.time()
        while (time.time() - start_time) < timeout:
            # Check if message received (RX0IF flag)
            canintf = self._read_byte(MCP2515Registers.CANINTF)
            if canintf & 0x01:  # RX0IF set
                # Read CAN ID
                sidh = self._read_byte(MCP2515Registers.RXB0SIDH)
                sidl = self._read_byte(MCP2515Registers.RXB0SIDL)

                # Check if extended ID
                is_extended = bool(sidl & 0x08)

                if is_extended:
                    # Extended ID (29-bit)
                    eid8 = self._read_byte(MCP2515Registers.RXB0EID8)
                    eid0 = self._read_byte(MCP2515Registers.RXB0EID0)
                    can_id = (
                        (sidh << 21)
                        | ((sidl & 0xE0) << 13)
                        | ((sidl & 0x03) << 16)
                        | (eid8 << 8)
                        | eid0
                    )
                else:
                    # Standard ID (11-bit)
                    can_id = (sidh << 3) | ((sidl >> 5) & 0x07)

                # Read DLC
                dlc = self._read_byte(MCP2515Registers.RXB0DLC) & 0x0F

                # Read data bytes
                data = []
                for i in range(dlc):
                    data.append(self._read_byte(MCP2515Registers.RXB0D0 + i))

                # Clear RX0IF flag
                self._write_byte(MCP2515Registers.CANINTF, 0x00)

                # Re-enable RX interrupt
                self._write_byte(MCP2515Registers.CANINTE, 0x01)

                msg = CANMessage(
                    can_id=can_id,
                    data=bytes(data),
                    dlc=dlc,
                    is_extended=is_extended,
                )

                logger.debug(
                    f"RX: ID=0x{can_id:03X if not is_extended else can_id:08X} "
                    f"DLC={dlc} Data={' '.join(f'{b:02X}' for b in data)}"
                )

                return msg

            # Small delay to avoid busy-waiting
            time.sleep(0.001)

        return None

    def check_rx_status(self) -> int:
        """Check RX buffer status.

        :return: RX status byte (bit 0 = RX0 has message, bit 1 = RX1 has message).
        """
        if self.spi is None:
            return 0
        return self._read_byte(MCP2515Registers.CANINTF) & 0x03

    def get_error_flags(self) -> int:
        """Get error flag register.

        :return: EFLG register value.
        """
        if self.spi is None:
            return 0
        return self._read_byte(MCP2515Registers.EFLG)

    def close(self) -> None:
        """Close SPI connection.

        :return: None
        """
        if self.spi is not None:
            try:
                self.spi.close()
                logger.info("MCP2515 SPI connection closed")
            except Exception as e:
                logger.warning(f"Error closing SPI: {e}")
            finally:
                self.spi = None
                self.connected = False

    def __enter__(self) -> "MCP2515":
        """Context manager entry.

        :return: Self.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit.

        :param exc_type: Exception type.
        :param exc_val: Exception value.
        :param exc_tb: Exception traceback.
        :return: None
        """
        self.close()

    def __del__(self) -> None:
        """Destructor.

        :return: None
        """
        self.close()
