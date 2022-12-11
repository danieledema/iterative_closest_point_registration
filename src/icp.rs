use std::f32::INFINITY;

use rayon::prelude::*;

use nalgebra::{Isometry3, Matrix3, Point3, Quaternion, Rotation3, Vector3, SVD};
use navigation_core::{
    domain::{timestamp::TimeStamp, transformation::Transformation},
    ports::{odometry::Odometry, sensor_input::SensorData},
};

use kiddo::distance::squared_euclidean;
use kiddo::KdTree;

use crate::pointcloud::PointCloudXYZ;

pub struct IterativeClosestPoint {
    rmse_threshold: f32,
    max_iterations: u32,
    last_pointcloud: Option<PointCloudXYZ>,
}

impl IterativeClosestPoint {
    pub fn new(rmse_threshold: f32, max_iterations: u32) -> Self {
        IterativeClosestPoint {
            rmse_threshold,
            max_iterations,
            last_pointcloud: None,
        }
    }

    fn calc_odometry(
        &self,
        pointcloud_pre: &PointCloudXYZ,
        pointcloud_post: &PointCloudXYZ,
    ) -> Transformation {
        let mut last_rmse = INFINITY;

        let transform = Isometry3::identity();
        let mut trans = Transformation {
            timestamp_from: pointcloud_post.timestamp,
            timestamp_to: pointcloud_pre.timestamp,
            frame_from: pointcloud_post.sensor_id.clone(),
            frame_to: pointcloud_pre.sensor_id.clone(),
            transform,
        };

        for _ in 0..self.max_iterations {
            let matches = self.find_matches(pointcloud_pre, pointcloud_post);
            trans = self.register_points(pointcloud_pre, pointcloud_post, matches);
            let transformed_pc = pointcloud_pre.transform(&trans);
            let rmse = self.compute_rmse(&transformed_pc, &pointcloud_post);

            if (rmse - last_rmse).abs() < self.rmse_threshold {
                break;
            }
            last_rmse = rmse;
        }
        trans
    }

    fn register_points(
        &self,
        pointcloud_pre: &PointCloudXYZ,
        pointcloud_post: &PointCloudXYZ,
        matches: Vec<(usize, usize)>,
    ) -> Transformation {
        let mean_pre = calc_mean(&pointcloud_pre.data);
        let mean_post = calc_mean(&pointcloud_post.data);

        let pp_pre = subtract_mean(&pointcloud_pre.data, mean_pre);
        let pp_post = subtract_mean(&pointcloud_post.data, mean_post);

        let w: Matrix3<f32> = matches
            .iter()
            .map(|(i, j)| pp_pre[*i].coords * pp_post[*j].coords.transpose())
            .sum();

        let svd = SVD::new(w, true, true);
        let u = svd.u.unwrap();
        let mut vt = svd.v_t.unwrap();

        let mut r = (u * vt).transpose();
        if r.determinant() < 0. {
            let mut vtrow = vt.row_mut(2);
            vtrow[0] = -vtrow[0];
            vtrow[1] = -vtrow[1];
            vtrow[2] = -vtrow[2];

            r = (u * vt).transpose();
        }
        let r = Rotation3::from_matrix(&r);
        let t = nalgebra::Translation {
            vector: mean_post - r * mean_pre,
        };
        let transform = Isometry3::from_parts(t, r.into());

        Transformation {
            timestamp_from: pointcloud_post.timestamp,
            timestamp_to: pointcloud_pre.timestamp,
            frame_from: pointcloud_post.sensor_id.clone(),
            frame_to: pointcloud_pre.sensor_id.clone(),
            transform,
        }
    }

    fn compute_rmse(&self, pointcloud1: &PointCloudXYZ, poincloud2: &PointCloudXYZ) -> f32 {
        pointcloud1
            .data
            .iter()
            .zip(poincloud2.data.iter())
            .map(|(x, y)| (x - y).dot(&(x - y)))
            .map(|x| x.sqrt())
            .sum()
    }

    fn find_matches(
        &self,
        pointcloud_pre: &PointCloudXYZ,
        pointcloud_post: &PointCloudXYZ,
    ) -> Vec<(usize, usize)> {
        let mut kdtree = KdTree::new();
        for (i, p) in pointcloud_post.data.iter().enumerate() {
            kdtree.add(&[p[0], p[1], p[2]], i).unwrap();
        }

        pointcloud_pre
            .data
            .par_iter()
            .enumerate()
            .map(|(i, p)| {
                let p = [p[0], p[1], p[2]];
                let nearest = kdtree.nearest(&p, 1, &squared_euclidean).unwrap();
                (i, nearest[0].1.clone())
            })
            .collect()
    }
}

impl Odometry for IterativeClosestPoint {
    fn get_odometry(&mut self, input_now: &SensorData) -> Transformation {
        let input_now = PointCloudXYZ::from(input_now);

        let transform: Transformation;
        if let Some(input_pre) = &self.last_pointcloud {
            transform = self.calc_odometry(&input_pre, &input_now);
        } else {
            transform = Transformation::new(
                TimeStamp::from(input_now.timestamp),
                TimeStamp::from(input_now.timestamp),
                &input_now.sensor_id,
                &input_now.sensor_id,
                Vector3::new(0., 0., 0.),
                Quaternion::new(1., 0., 0., 0.),
            );
        }

        self.last_pointcloud = Some(input_now.clone());
        transform
    }
}

fn calc_mean(points: &Vec<Point3<f32>>) -> Point3<f32> {
    Point3::<f32>::from(
        points.iter().map(|x| x.coords).sum::<Vector3<f32>>() / (points.len() as f32),
    )
}

fn subtract_mean(points: &Vec<Point3<f32>>, mean: Point3<f32>) -> Vec<Point3<f32>> {
    points.iter().map(|x| Point3::from(x - mean)).collect()
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn test_mean() {
        let points = vec![Point3::new(0., 0., 0.), Point3::new(2., 2., 2.)];
        let mean = calc_mean(&points);
        assert_relative_eq!(mean[0], 1.);
        assert_relative_eq!(mean[1], 1.);
        assert_relative_eq!(mean[2], 1.);
    }

    #[test]
    fn test_sub_mean() {
        let points = vec![Point3::new(0., 0., 0.), Point3::new(2., 2., 2.)];
        let mean = calc_mean(&points);
        let new_points = subtract_mean(&points, mean);

        assert_relative_eq!(new_points[0][0], -1.);
        assert_relative_eq!(new_points[0][1], -1.);
        assert_relative_eq!(new_points[0][2], -1.);

        assert_relative_eq!(new_points[1][0], 1.);
        assert_relative_eq!(new_points[1][1], 1.);
        assert_relative_eq!(new_points[1][2], 1.);
    }
}
