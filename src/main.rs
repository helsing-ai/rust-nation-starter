use std::time::Duration;

use hs_hackathon::cheats::angles::Vector;
use hs_hackathon::cheats::approaching::Hint;
use hs_hackathon::cheats::positioning::Position;
use hs_hackathon::cheats::TeamColors;
use hs_hackathon::{cheats, prelude::*};

const CAR: Color = Color::Red;
const TARGET: Color = Color::Blue;
const ANGLE_THRESHOLD: u32 = 10;

struct MapState {
    car: Position,
    target: Position,
}

impl MapState {
    pub async fn infer(drone: &mut Camera) -> eyre::Result<Self> {
        /* SOLUTION CODE */

        loop {
            let frame = drone.snapshot().await?;

            let leds = hackathon::vision::detect(&frame, &LedDetectionConfig::default())?;

            let Some(car) = leds.iter().find(|led| led.color == CAR) else {
                continue;
            };

            let Some(target) = leds.iter().find(|led| led.color == TARGET) else {
                continue;
            };

            return Ok(MapState {
                car: car.bbox.into(),
                target: target.bbox.into(),
            });
        }
    }

    async fn car_orientation(
        current: Position,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<Vector> {
        const ORIENTATION_DURATION: Duration = Duration::from_millis(100);

        // 1. move a bit forward
        wheels.set(Angle::straight()).await?;
        motor
            .move_for(Velocity::forward(), ORIENTATION_DURATION)
            .await?;

        // 2. measure the delta
        let MapState { car: delta, .. } = MapState::infer(drone).await?;

        // 3. compute the angle
        Ok(Vector::from((current, delta)))
    }
}

#[derive(Debug)]
enum State {
    /// Turn the cars direction by doing consecutive front and back movements
    /// until the angle between the cars orientation and the target converges to be under
    /// a specified threshold
    Turning,
    /// Approach the car by doing incremental actions of approaching and measuring interleaved.
    /// So we approach the target a bit, measure if we decreased the distance, if yes repeat, if no
    /// then calibrate. We do this until we hit the target.
    Approaching,
    /// Simply idling on the target and identifying when the target moves away from our current
    /// position.
    Idle,
}

impl State {
    async fn execute(
        &mut self,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<()> {
        /* SOLUTION CODE */

        match self {
            State::Turning => 'turning: loop {
                const TURNING_DURATION: Duration = Duration::from_secs(2);

                let pre = MapState::infer(drone).await?;

                let orientation_offset_angle = {
                    let car = MapState::car_orientation(pre.car, drone, motor, wheels).await?;
                    let target = Vector::from((pre.car, pre.target));

                    car.angle(target)
                };

                wheels.set(Angle::straight()).await?;
                motor
                    .move_for(Velocity::backward(), TURNING_DURATION)
                    .await?;

                wheels
                    .set({
                        let tolerance = (ANGLE_THRESHOLD / 2) as f64;

                        match orientation_offset_angle {
                            a if a < -tolerance => Angle::left(),
                            a if a > tolerance => Angle::right(),
                            _ => Angle::straight(),
                        }
                    })
                    .await?;

                motor
                    .move_for(Velocity::forward(), TURNING_DURATION)
                    .await?;
                wheels.set(Angle::straight()).await?;

                let new_offset_angle = {
                    let new = MapState::infer(drone).await?;
                    let car = MapState::car_orientation(new.car, drone, motor, wheels).await?;
                    let target = Vector::from((new.car, new.target));

                    car.angle(target)
                };

                // 1. determine whether the orientation angle of the car is below an a threshold
                if (new_offset_angle.abs() as u32) < ANGLE_THRESHOLD {
                    *self = State::Approaching
                }

                // 2. verify if we have come closer to the target
                if new_offset_angle.abs() > orientation_offset_angle {
                    tracing::warn!("delta angle between car orientation and target has increased");
                }

                continue 'turning;
            },
            State::Approaching => {
                let hint = cheats::approaching::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = match hint {
                    Hint::TargetWasHit => Self::Idle,
                    Hint::OrientationIsOff => Self::Turning,
                };
            }
            State::Idle => {
                cheats::idling::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = Self::Turning;
            }
        }

        Ok(())
    }
}

#[hackathon::main]
async fn main() -> eyre::Result<()> {
    let mut wheels = WheelOrientation::new().await?;
    let mut motor = MotorSocket::open().await?;
    let mut drone = Camera::connect().await?;

    let mut machine = State::Turning;

    loop {
        machine.execute(&mut drone, &mut motor, &mut wheels).await?;
        tracing::debug!("{:?}", machine);
    }
}
