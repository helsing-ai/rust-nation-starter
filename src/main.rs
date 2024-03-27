mod cheats;

use std::f32::consts::PI;
use std::time::Duration;

use hs_hackathon::prelude::*;

use cheats::angles::Vector;
use cheats::approaching::Hint;
use cheats::positioning::Position;
use cheats::TeamColors;

const CAR: Color = Color::Blue;
const TARGET: Color = Color::Green;

#[allow(unused)]
struct MapState {
    car: Position,
    target: Position,
}

#[allow(unused)]
impl MapState {
    pub async fn infer(drone: &mut Camera) -> eyre::Result<Self> {
        let frame = drone.snapshot().await?;
        let leds = detect(&frame.0, &LedDetectionConfig::default())?;

        let target = leds.iter().find(|led| led.color == TARGET).expect("Found the target");
        let car = leds.iter().find(|led| led.color == CAR).expect("Found the car");

        let target = Position {
            x: target.bbox.x_min(),
            y: target.bbox.y_min()
        };

        let car = Position {
            x: car.bbox.x_min(),
            y: car.bbox.y_min(),
        };

        Ok(Self {
            car,
            target,
        })
    }

    async fn car_orientation(
        current: Position,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<Vector> {
        unimplemented!()
    async fn angle_to_target(
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<Angle> {
        const APPROACHING_DURATION: Duration = Duration::from_secs(2);

        let old = Self::infer(drone).await?;
        motor
            .move_for(Velocity::forward(), APPROACHING_DURATION)
            .await?;
        let new = Self::infer(drone).await?;

        let target_vector = Vector::from((new.target, new.car));
        let car_vector = Vector::from((old.car, new.car));

        Angle::try_from(car_vector.angle(target_vector) as f32)
    }
}

#[derive(Debug)]
#[allow(unused)]
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
        match self {
            State::Turning => loop {
                let angle = MapState::angle_to_target(drone, motor, wheels).await?;
                wheels.set(angle).await?;

                *self = Self::Approaching;
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

#[hs_hackathon::main]
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
