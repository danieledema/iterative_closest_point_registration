use nalgebra::Point3;
use navigation_core::{
    domain::{timestamp::TimeStamp, transformation::Transformation},
    ports::sensor_input::SensorData,
};

#[derive(Clone, Debug)]
pub struct PointCloudXYZ {
    pub timestamp: TimeStamp,
    pub sensor_id: String,
    pub data: Vec<Point3<f32>>,
}

impl PointCloudXYZ {
    pub fn transform(&self, transf: &Transformation) -> PointCloudXYZ {
        PointCloudXYZ {
            timestamp: self.timestamp,
            sensor_id: self.sensor_id.clone(),
            data: self
                .data
                .iter()
                .map(|x| transf.transform.transform_point(x))
                .collect(),
        }
    }
}

impl From<&SensorData> for PointCloudXYZ {
    fn from(value: &SensorData) -> Self {
        let data: Vec<Point3<f32>> = match value.sensor_type.as_str() {
            "pointcloud_xyzi" => {
                let data_vec: Vec<(f32, f32, f32, f32)> =
                    serde_json::from_str(&value.data).unwrap();
                data_vec
                    .iter()
                    .map(|x| Point3::new(x.0, x.1, x.2))
                    .collect()
            }
            _ => panic!(
                "Sensor type {} not supported for conversion",
                value.sensor_type
            ),
        };

        PointCloudXYZ {
            timestamp: value.timestamp,
            sensor_id: value.sensor_id.clone(),
            data,
        }
    }
}
