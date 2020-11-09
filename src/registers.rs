use core::convert::TryFrom;

pub enum Register {
    TargetSpeed,
    Odometry,
    RampFrequency,
    RampStep,
    FaultStatus,
}

impl TryFrom<u8> for Register {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x10 => Ok(Self::TargetSpeed),
            0x20 => Ok(Self::Odometry),
            0x30 => Ok(Self::RampFrequency),
            0x40 => Ok(Self::RampStep),
            0x50 => Ok(Self::FaultStatus),
            _ => Err(()),
        }
    }
}
