use super::TeamColors;
use hs_hackathon::drone::Camera;
use hs_hackathon::prelude::*;
use hs_hackathon::vision::{detect, BoundingBox, LedDetectionConfig};

/// A utility inference function
#[doc(hidden)]
pub(crate) async fn infer(
    colors: &TeamColors,
    camera: &Camera,
) -> eyre::Result<(BoundingBox, BoundingBox)> {
    loop {
        let frame = camera.snapshot().await?;

        let leds = detect(&frame, &LedDetectionConfig::default())?;

        let Some(car) = leds.iter().find(|led| led.color == colors.car) else {
            continue;
        };

        let Some(target) = leds.iter().find(|led| led.color == colors.target) else {
            continue;
        };

        Ok((car.bbox, target.bbox))
    }
}
