#include "ros/ros.h"
#include "firefly_mapping/ImageWithPose.h"
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/UInt8MultiArray.h>
#include <chrono>


class FireflyMapping {

public:
    FireflyMapping() {
        image_sub = nh.subscribe("image_to_project", 1000, &FireflyMapping::project_image, this);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("observed_firemap", 10);

        K_inv << 1.0/fx,  0.0,    -cx/fx,
                 0.0,     1.0/fy, -cy/fy,
                 0.0,     0.0,     1.0;

        map.header.frame_id = "world";
        map.info.resolution = 0.5;
        map.info.width = 400; //Number of Cells
        map.info.height = 400; //Number of Cells
        map.info.origin.position.x = -100; //In meters
        map.info.origin.position.y = -100; //In meters
        map.data = std::vector<std::int8_t> (400*400, 50); // Initialize map to 50 percent certainty


    }

private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Publisher map_pub;

    nav_msgs::OccupancyGrid map;

    Eigen::Vector3d ground_normal{0, 0, 1}; //Should point up from ground - if pointing into ground, will cause errors
    float ground_offset = 0;

    float fx = 411.3366;
    float fy = 412.6712;
    float cx = 320.4049;
    float cy = 251.967;

    float resolution = 0.5;
    float minX = -100;
    float maxX = 100;
    float minY = -100;
    float maxY = 100;

    float fpr = 0.05; // False positives divided by all negative cases
    float fnr = 0.05; // False negatives divided by all positive cases
    float tpr = 1 - fnr; // True positives divided by all positives
    float tnr = 1 - fpr; // True negatives divided by all negatives

    Eigen::Matrix3d K_inv;

    void project_image(const firefly_mapping::ImageWithPose& msg) {
        auto start = std::chrono::high_resolution_clock::now();
        Eigen::Quaterniond cam_quat;
        tf::quaternionMsgToEigen(msg.pose.orientation, cam_quat);

        Eigen::Matrix3d pixelToRay = cam_quat.matrix() * K_inv;
        Eigen::Vector3d camCenter {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};

        float camCenterGroundDist = ground_offset - ground_normal.dot(camCenter);

        std::cout << "Projecting and filtering!" << std::endl;

        for (size_t i = 0; i < msg.image.width; i++) {
            for (size_t j = 0; j < msg.image.height; j++) {
                Eigen::Vector3d pixelHomogenous{i, j, 1};
                Eigen::Vector3d rayDir = pixelToRay * pixelHomogenous;

                float rayPerpendicularComponent = ground_normal.dot(rayDir);
                if (rayPerpendicularComponent >= 0) {
                    continue;
                }

                float rayLambda = camCenterGroundDist / rayPerpendicularComponent;

                Eigen::Vector3d intersect = camCenter + rayLambda * rayDir;

                if (intersect(0) > maxX
                    || intersect(0) < minX
                    || intersect(1) > maxY
                    || intersect(1) < minY) {
                    continue;
                }

                size_t gridRow = (size_t) ((intersect(1)-minY)/resolution);
                size_t gridCol = (size_t) ((intersect(0)-minX)/resolution);

                int mapBin = gridCol + gridRow * map.info.width;
                uint8_t pixelValue = msg.image.data[i + j * msg.image.width];
                std::int8_t prior = map.data[mapBin];

                if (pixelValue == 0) {
                    float probOfNegative = (fnr * prior + tnr * (100 - prior))/100.0;
                    uint8_t posterior = (fnr/probOfNegative) * prior;
                    map.data[mapBin] = posterior;
                }
                else {
                    float probOfPositive = (tpr * prior + fpr * (100 - prior))/100.0;
                    uint8_t posterior = (tpr/probOfPositive) * prior;
                    map.data[mapBin] = posterior;
                }

            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Projection of single image took: " << duration.count() << " milliseconds" << std::endl;

        map_pub.publish(map);
        return;
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "firefly_mapping");
    FireflyMapping node;
    ros::spin();
    return 0;
}