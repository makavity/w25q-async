//! An [`embedded-hal-async`]-based W25Q SPI flash chip driver.
//!
//! Contributions are welcome!
//!
//! [`embedded-hal-async`]: https://docs.rs/embedded-hal-async/

#![warn(missing_debug_implementations)]
#![warn(missing_docs)]
#![cfg_attr(not(test), no_std)]

mod error;
pub use crate::error::Error;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

/// 3-Byte JEDEC manufacturer and device identification.
#[derive(Copy, Clone)]
pub struct JedecId {
    /// The manufacturer ID.
    pub manufacturer_id: u8,

    /// The device ID.
    pub device_id: u16,
}

impl JedecId {
    /// Build an Identification from JEDEC ID bytes.
    pub fn parse(buf: &[u8]) -> Option<Self> {
        // Example response for Cypress part FM25V02A:
        // 7F 7F 7F 7F 7F 7F C2 22 08  (9 bytes)
        // 0x7F is a "continuation code", not part of the core manufacturer ID
        // 0xC2 is the company identifier for Cypress (Ramtron)

        let start = buf.iter().position(|&x| x != 0x7F)?;
        let buf = &buf[start..];
        if buf.len() < 3 {
            return None;
        }

        let manufacturer_id = buf[0];
        let device_id = u16::from_be_bytes([buf[1], buf[2]]);

        Some(Self {
            manufacturer_id,
            device_id,
        })
    }
}

impl core::fmt::Debug for JedecId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let id = u32::from(self.manufacturer_id) << 16 | u32::from(self.device_id);
        f.debug_tuple("Identification")
            .field(&format_args!("0x{id:06X}"))
            .finish()
    }
}

#[repr(u8)]
#[allow(unused)] // TODO support more features
enum Opcode {
    /// Read the 8-bit legacy device ID.
    ReadDeviceId = 0xAB,
    /// Read the 8-bit manufacturer and device IDs.
    ReadMfDId = 0x90,
    /// Read 16-bit manufacturer ID and 8-bit device ID.
    ReadJedecId = 0x9F,
    /// Set the write enable latch.
    WriteEnable = 0x06,
    /// Clear the write enable latch.
    WriteDisable = 0x04,
    /// Read the 8-bit status register.
    ReadStatus = 0x05,
    /// Write the 8-bit status register. Not all bits are writeable.
    WriteStatus = 0x01,
    Read = 0x03,
    PageProg = 0x02, // directly writes to EEPROMs too
    SectorErase = 0x20,
    BlockErase = 0xD8,
    ChipErase = 0xC7,
    PowerDown = 0xB9,
}

/// Status of the flash memory.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Status {
    raw: u8,
}

impl Status {
    /// Interpret a raw [`u8`] as a `Status`.
    pub fn from_raw(raw: u8) -> Self {
        Self { raw }
    }

    /// Get the raw [`u8`] value of the `Status`.
    pub fn as_raw(self) -> u8 {
        self.raw
    }

    /// Check if the flash memory is busy processing an erase or program command.
    pub fn busy(&self) -> bool {
        self.raw & 0x01 != 0
    }

    /// Check if the write enable latch is set.
    ///
    /// The write enable latch must be set in order to execute an erase or program command.
    pub fn write_enable_latch(&self) -> bool {
        self.raw & 0x02 != 0
    }

    /// Get the protection bits of the memory.
    ///
    /// Each bit correponds to a region of memory that can be protected.
    /// See the datasheet of your flash memory for more details.
    pub fn protection(&self) -> u8 {
        (self.raw >> 2) & 0b111
    }

    /// TODO: what exactly is this?
    pub fn status_register_write_disable(&self) -> u8 {
        self.raw & 0x80
    }
}

impl core::fmt::Debug for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct(core::any::type_name::<Self>())
            .field("busy", &self.busy())
            .field("write_enable_latch", &self.write_enable_latch())
            .field("protection", &format_args!("0b{:03b}", self.protection()))
            .field("status_register_write_disable", &self.status_register_write_disable())
            .finish()
    }
}

/// Information about the flash memory.
#[derive(Debug)]
pub struct FlashInfo {
    /// The chip ID.
    pub id: JedecId,

    /// The page size (and alignment).
    pub page_size: u16,

    /// The sector size (and alignment).
    pub sector_size: u32,

    /// The number of pages in a sector.
    pub page_count: u32,

    /// The number of sectors in a block.
    pub sector_count: u32,

    /// The size of a block.
    pub block_size: u32,

    /// The number of blocks in the memory.
    pub block_count: u32,

    /// The total memory capacity in KiB.
    pub capacity_kb: u32,
}

/// Driver for W25Q-series SPI Flash chips.
#[derive(Debug)]
pub struct Flash<SPI> {
    spi: SPI,
}

// for multiple SPI use: https://crates.io/crates/shared-bus-rtic

impl<SPI: SpiDevice> Flash<SPI> {
    /// Creates a new W25Q-series flash driver.
    pub async fn new(spi: SPI) -> Result<Self, Error<SPI::Error>> {
        let mut this = Self { spi };

        let status = this.read_status().await?;

        // Here we don't expect any writes to be in progress, and the latch must also be deasserted.
        if status.busy() || status.write_enable_latch() {
            return Err(Error::UnexpectedStatus);
        }

        Ok(this)
    }

    /// Reads the JEDEC manufacturer/device identification.
    pub async fn read_jedec_id(&mut self) -> Result<JedecId, Error<SPI::Error>> {
        // Optimistically read 12 bytes, even though some identifiers will be shorter
        let mut buf: [u8; 12] = [0; 12];
        buf[0] = Opcode::ReadJedecId as u8;
        self.spi_transfer_inplace(&mut buf).await?;

        // Skip buf[0] (SPI read response byte)
        JedecId::parse(&buf[1..])
            .ok_or_else(|| Error::InvalidResponse)
    }

    /// Reads the status register.
    pub async fn read_status(&mut self) -> Result<Status, Error<SPI::Error>> {
        let mut buf = [Opcode::ReadStatus as u8, 0];
        self.spi_transfer_inplace(&mut buf).await?;

        Ok(Status { raw: buf[1] })
    }

    /// Get the device information.
    pub async fn get_device_info(&mut self) -> Result<FlashInfo, Error<SPI::Error>> {
        let id = self.read_jedec_id().await?;

        // TODO: Why ignore manufacturer ID and the high byte of the device ID?
        let block_count = match id.device_id & 0xFF {
            0..=0x10 => 0, // TODO: Are these really unused?
            0x11 => 2,    // W25Q10
            0x12 => 4,    // W25Q20
            0x13 => 8,    // W25Q40
            0x14 => 16,   // W25Q80
            0x15 => 32,   // W25Q16
            0x16 => 64,   // W25Q32
            0x17 => 128,  // W25Q64
            0x18 => 256,  // W25Q128
            0x19 => 512,  // W25Q256
            0x1A..=0x1F => 0, // TODO: Are these really unused?
            0x20 => 1024, // W25Q512
            0x21.. => 0, // TODO: Are these really unused?
        };

        let sector_size = 0x1000;
        let page_size = 256;
        let block_size = sector_size * 16;
        Ok(FlashInfo {
            id,
            page_size,
            sector_size,
            sector_count: block_count * 16,
            page_count: (block_count * block_size) / u32::from(page_size),
            block_size,
            block_count,
            capacity_kb: (block_size * block_count) / 1024,
        })
    }

    /// Set the write-enable bit on the flash device.
    async fn write_enable(&mut self) -> Result<(), Error<SPI::Error>> {
        self.spi_write(&[Opcode::WriteEnable as u8]).await
    }

    /// Wait for the BUSY status bit to clear.
    async fn wait_done(&mut self) -> Result<(), Error<SPI::Error>> {
        while self.read_status().await?.busy() {
            // TODO: Consider sleeping here.
        }
        Ok(())
    }

    /// Enter power down mode.
    ///
    /// Datasheet, 8.2.35: Power-down:
    /// Although  the  standby  current  during  normal  operation  is  relatively  low,  standby  current  can  be  further
    /// reduced  with  the  Power-down  instruction.  The  lower  power  consumption  makes  the  Power-down
    /// instruction especially useful for battery powered applications (See ICC1 and ICC2 in AC Characteristics).
    /// The instruction is initiated by driving the /CS pin low and shifting the instruction code “B9h” as shown in
    /// Figure 44.
    ///
    /// The /CS pin must be driven high after the eighth bit has been latched. If this is not done the Power-down
    /// instruction will not be executed. After /CS is driven high, the power-down state will entered within the time
    /// duration of tDP (See AC Characteristics). While in the power-down state only the Release Power-down /
    /// Device ID (ABh) instruction, which restores the device to normal operation, will be recognized. All other
    /// instructions  are  ignored.  This  includes  the  Read  Status  Register  instruction,  which  is  always  available
    /// during normal operation. Ignoring all but one instruction makes the Power Down state a useful condition
    /// for  securing maximum  write protection. The  device  always  powers-up  in the  normal  operation with  the
    /// standby current of ICC1.
    pub async fn power_down(&mut self) -> Result<(), Error<SPI::Error>> {
        self.spi_write(&[Opcode::PowerDown as u8]).await
    }

    /// Exit power down mode.
    ///
    /// Datasheet, 8.2.36: Release Power-down:
    /// The Release from Power-down /  Device ID instruction is  a multi-purpose instruction. It can be used to
    /// release the device from the power-down state, or obtain the devices electronic identification (ID) number.
    /// To  release the device  from  the  power-down state,  the instruction  is  issued by driving the  /CS  pin low,
    /// shifting the instruction code “ABh” and driving /CS high as shown in Figure 45. Release from power-down
    /// will  take  the  time  duration  of  tRES1  (See  AC  Characteristics)  before  the  device  will  resume  normal
    /// operation  and  other  instructions  are  accepted.  The  /CS  pin  must  remain  high  during  the  tRES1  time
    /// duration.
    ///
    /// Note: must manually delay after running this, IOC
    pub async fn power_up<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<SPI::Error>> {
        // Same command as reading ID.. Wakes instead of reading ID if not followed by 3 dummy bytes.
        self.spi_write(&[Opcode::ReadDeviceId as u8]).await?;

        // Table 9.7: AC Electrical Characteristics: tRES1 = max 3us.
        // TODO: Is this chip specific?
        delay.delay_us(6).await;

        Ok(())
    }

    /// Reads memory into `buf`, starting at `addr`.
    ///
    /// The read address must be aligned according to the requirements of the actual flash chip.
    /// See the datasheet of your specific chip for details.
    ///
    /// Address is truncated to 24 bits before being transmitted.
    /// See the datasheet of your flash chip to know what happens when the address exceeds the available memory.
    pub async fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        if buf.is_empty() {
            return Ok(());
        }

        let addr = addr.to_le_bytes();
        let cmd_buf = [
            Opcode::Read as u8,
            addr[2],
            addr[1],
            addr[0],
        ];

        let mut transaction = [
            embedded_hal_async::spi::Operation::Write(&cmd_buf),
            embedded_hal_async::spi::Operation::Read(buf),
        ];
        self.spi.transaction(&mut transaction)
            .await
            .map_err(Error::Spi)
    }

    /// Erase a sector of memory.
    ///
    /// The address must be sector aligned.
    pub async fn erase_sector(&mut self, addr: u32) -> Result<(), Error<SPI::Error>> {
        self.write_enable().await?;

        let addr = addr.to_be_bytes();
        let cmd_buf = [
            Opcode::SectorErase as u8,
            addr[2],
            addr[1],
            addr[0],
        ];
            self.spi_write(&cmd_buf).await?;
            self.wait_done().await?;
        Ok(())
    }

    /// Program (write) bytes to the memory.
    ///
    /// That start address must be page aligned,
    /// and for most chips, the bytes being written to must have been erased
    /// (see the datasheet of your specific chip for details).
    pub async fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        let mut current_addr = addr;
        for chunk in data.chunks(256) {
            self.write_enable().await?;

            let addr_bytes = current_addr.to_be_bytes();
            current_addr += 256;
            let cmd_buf = [
                Opcode::PageProg as u8,
                addr_bytes[2],
                addr_bytes[1],
                addr_bytes[0],
            ];

            let mut transaction = [
                embedded_hal_async::spi::Operation::Write(&cmd_buf),
                embedded_hal_async::spi::Operation::Write(chunk)
            ];
            self.spi.transaction(&mut transaction)
                .await
                .map_err(Error::Spi)?;
            self.wait_done().await?;
        }
        Ok(())
    }

    /// Erase a single block.
    pub async fn erase_block(&mut self, addr: u32) -> Result<(), Error<SPI::Error>> {
        self.write_enable().await?;

        let addr = addr.to_be_bytes();
        let cmd_buf = [
            Opcode::BlockErase as u8,
            addr[2],
            addr[1],
            addr[0],
        ];
        self.spi_write(&cmd_buf).await?;
        self.wait_done().await
    }

    /// Erase the entire memory.
    pub async fn erase_all(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write_enable().await?;
        self.spi_write(&[Opcode::ChipErase as u8]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Perform an in-place SPI tranfer.
    async fn spi_transfer_inplace(&mut self, buffer: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        self.spi.transfer_in_place(buffer).await.map_err(Error::Spi)
    }

    /// Perform an in-place SPI tranfer.
    async fn spi_write(&mut self, buffer: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.spi.write(buffer).await.map_err(Error::Spi)
    }
}

#[cfg(test)]
mod tests {
    use assert2::assert;
    use super::*;

    #[test]
    fn test_decode_jedec_id() {
        let cypress_id_bytes = [0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x08];
        assert!(let Some(ident) = JedecId::parse(&cypress_id_bytes));
        assert!(ident.manufacturer_id == 0xC2);
        assert!(ident.device_id == 0x2208);
    }
}
