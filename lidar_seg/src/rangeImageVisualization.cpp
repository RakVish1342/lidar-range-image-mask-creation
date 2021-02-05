/**
 *Creating and viewing range images 
 *https://pointclouds.org/documentation/tutorials/range_image_visualization.html
 *https://pointclouds.org/documentation/tutorials/range_image_creation.html
 *Visualization: https://pcl.readthedocs.io/en/latest/range_image_visualization.html
 *Save Range Img as PNG: http://www.pcl-users.org/how-to-save-range-image-td4042471.html
 */

/**
* Notes to self:

*Processing is done on the segmented and labelled ASCII format clouds saved from cloudcompare
*Points labelled isTree==1 are part of the tree while the others are not
*Non-tree points need to be suppressed/removed. This is done by divividing all such points by 1000
*Such division ensures that all such non-tree points appear "very close" to the lidar, due to which
the range image created will place such points to be very dark or near black.
*Remaining non-tree points will retain original range image colour and can be thresholded out using opencv
*Reason for division by 1000 rather than simply disregarding such points: The final point cloud created 
needs to have a full size of vert 32 odd pixels and horizontal 360 odd pixels. If all non-tree points are 
simply igonred, then the size of the range image created will be cropped only to include the tree image.
To keep all the points in their ordered locations (therefore retaining the full lidar range image) 
but only highlighting the tree points, all non-tree points are suppressed by dividing by 1000

*Sample cloud88_88.000000.pcd:
*The above had an issue. So the fix was to invert the solution proposed above:
*There were some sections of the flight where the UAV got pretty close to the tree. In these cases, the 
tree points which were not to be suppressed were "auto-supressed" since they already had a small value
by default.
*So, now all background non-tree points are being multiplied by 1000, so that in the normalized range
image all non-tree points appear coloured, but the tree points appear fully black.
*Then opencv just thresholds out all black/near black points which now represent the tree.


*Case where issues may arise:
*Issues related to size of the rangeImage not being maintained to the original intended size:
*If say the drone is tilted by a large angle, say 45 deg, then part of the lidar scan that shoots off into the sky
will never return. Maybe only half of the scan will be registered. 
*In this case, the range image might be reduced/cropped to only that half containing points which did actually return, 
causing the total size/dimension of the range image to be smaller than the expected 32x360 for example.
*Its just that in my simulation, there is always at least one ring which has all points/most points
returning, so a full 360 scan range image is generated.


*/

// --------------------------------------------------------------------------------------------
#include <filesystem>
#include <iostream>
#include <istream>
#include <sstream>
#include <unistd.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZL PointType;


// Set basic Range Image Parameters
// Setting such that image size of 30 x 360 is obtained. Approx 1 pixel for every degree in horiz and vert:
float angular_resolution_x = 1.005; // 1 deg resol from -180 deg to +180 deg
float angular_resolution_y = 0.85;  // 32 pixels in the vert...so that when encoder output of 1/8th value taken, we get an integer number
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

bool live_update = false;
bool bSuppressNonTree = true;
bool bOpenVisualizer = false;
unsigned int pngSaveTime = 0.2 * 1000000; // 0.2 sec
unsigned int headerLines = 2; // Number of header lines to ignore in pointcloud's ascii file...Ignore first two lines of headers if headers added from cloudcompare)

std::string basePath = "../pointclouds/";
std::string savePath1 = "3_png_rangeImg/";
std::string savePath2 = "4_png_rangeImg_forSeg";

// Output for help -h argument/option
void printUsage(const char *progName)
{
    std::cout << "\n\nUsage: " << progName << "[options]<scene.pcd>\n\n" <<
        "Options:\n" <<
        "-------------------------------------------\n" <<
        "-rx<float>  angular resolution in degrees (default " << angular_resolution_x << ")\n" <<
        "-ry<float>  angular resolution in degrees (default " << angular_resolution_y << ")\n" <<
        "-c<int>     coordinate frame (default " << (int) coordinate_frame << ")\n" <<
        "-l           live update - update the range image according to the selected view in the 3D viewer.\n" <<
        "-h           this help\n" <<
        "\n\n";
}

// Viewer object creation and param setting
void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose *Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() *Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() *Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
        look_at_vector[0], look_at_vector[1], look_at_vector[2],
        up_vector[0], up_vector[1], up_vector[2]);
}

int main(int argc, char **argv)
{

    // Parse Command Line Arguments
    //COMMAND: ./rangeImageVisualization OR ./rangeImageVisualization -l "path_to_pcd"
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, "-l") >= 0)
    {
        live_update = true;
        std::cout << "Live update is on.\n";
    }
    if (pcl::console::parse(argc, argv, "-rx", angular_resolution_x) >= 0)
        std::cout << "Setting angular resolution in x-direction to " << angular_resolution_x << "deg.\n";
    if (pcl::console::parse(argc, argv, "-ry", angular_resolution_y) >= 0)
        std::cout << "Setting angular resolution in y-direction to " << angular_resolution_y << "deg.\n";
    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        std::cout << "Using coordinate frame " << (int) coordinate_frame << ".\n";
    }
    angular_resolution_x = pcl::deg2rad(angular_resolution_x);
    angular_resolution_y = pcl::deg2rad(angular_resolution_y);

    pcl::PointCloud<PointType>::Ptr point_cloud_ptr1(new pcl::PointCloud < PointType>);
    pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr1;
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr2(new pcl::PointCloud < PointType>);
    pcl::PointCloud<PointType> &point_cloud_suppress = *point_cloud_ptr2;

    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

    int basePathLen = basePath.size();
    std::string folderName = "2_ascii";
    std::string fileExt = ".pcd";
    for (const auto &path: std::filesystem::directory_iterator(basePath + folderName))
    {
        // start with a fresh pointcloud
        point_cloud.clear();
        point_cloud_suppress.clear();

        // path = basePath + ascii/ 
        std::string filePath = path.path();
        std::string fileName(filePath.begin() + basePathLen + folderName.size(), filePath.end());   // exclude the /2_ascii folder path
        std::string fileNamePrefix(filePath.begin() + basePathLen + folderName.size(), filePath.end() - fileExt.size()); // Exclude ".pcd" file extension

        std::string line;
        ifstream myfile(filePath);
        if (myfile.is_open())
        {
            unsigned int ctr = headerLines;
            while(ctr)
            {
                std::string colNames, pcdLen;
                getline(myfile, colNames);
                ctr -= 1;
            }

            // pcl::Point<PointType> pt;
            pcl::PointXYZL pt;
            while (getline(myfile, line))
            {
                // cout << line << '\n';
                std::string x, y, z, isTree;
                std::stringstream ss(line);
                ss >> x >> y >> z >> isTree;
                // std::cout << std::stof(x) << " " << std::stof(y) << " " <<  std::stof(z) << " " <<  std::stoi(isTree) << endl;

                pt.x = stof(x);
                pt.y = stof(y);
                pt.z = stof(z);
                pt.label = stoi(isTree);

                // Add orig points to nominal pointcloud
                point_cloud.push_back(pt);

                // Suppressing or inflate points
                if (bSuppressNonTree)
                {
                    if (!pt.label)
                    {
                        // old logic was to suppress the non-tree points
                        // pt.x = pt.x / 1000;
                        // pt.y = pt.y / 1000;
                        // pt.z = pt.z / 1000;

                        // new logic to inflate non-tree points
                        pt.x = pt.x * 1000000;
                        pt.y = pt.y * 1000000;
                        pt.z = pt.z * 1000000;                        
                    }
                    else
                    {
                        // new logic to suppress tree points
                        pt.x = pt.x / 1000;
                        pt.y = pt.y / 1000;
                        pt.z = pt.z / 1000;
                    }
                }

                // Add scaled/suppressed points to suppressed pointcloud
                point_cloud_suppress.push_back(pt);

            }
            myfile.close();

            std::cout << "Point Cloud size is: " << point_cloud.size() << std::endl;

            // Original/Preferred way to set lidar viewing orientation for raw PCDs obtained/saved from the sensor. 
            // However this needs to be changed when pcd is saved as ascii from CloudCompare.
            /*
            // scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
            //                                                          point_cloud.sensor_origin_[1],
            //                                                          point_cloud.sensor_origin_[2])) *
            //                     Eigen::Affine3f (point_cloud.sensor_orientation_);

            // Eigen::Quaternion<_Scalar, _Options>::Quaternion  (const Scalar &w,
            //                                                       const Scalar &x,
            //                                                       const Scalar &y,
            //                                                        const Scalar &z 
                                                                   )   
            */

            // The way to correct/accound for the change in sensor orientation that CloudCompare imposes 
            // in the saved version of PCD/ASCII files as compared to what the lidar sensor would've saved.
            // To prevent this change in viewing angle in ClComp, might need to change some some 
            // 'viewport' settings in CloudComp I guess
            Eigen::Quaternion<float> cloudCompOrientation(1, 0, 0, 0);
            scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                    point_cloud.sensor_origin_[1],
                    point_cloud.sensor_origin_[2])) *
                Eigen::Affine3f(cloudCompOrientation);
        }
        else // File not found
        {
           std::cout << "File not found: " << filePath << std::endl;
           continue;
        }

        // Create RangeImage from the PointCloud
        float noise_level = 0.0;    // noise already added on the gazebo simulation.
        float min_range = 0.0f;    // Allow very small numbers since want to exclude suppressed non-tree points
        int border_size = 1;
        pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
        pcl::RangeImage &range_image = *range_image_ptr;


        // Viewed from orientation shown in the viewer
        /*
        // range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
        //                                   pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
        //                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
        */

        // Save the original range image
        // Viewed from lidar's perspective (LASER_FRAME as opposed to CAMERA_FRAME used in the live view of original code)
        range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
            pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
            scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
        float *ranges = range_image.getRangesArray();
        unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);
        std::string savePath = basePath + savePath1 + fileNamePrefix + ".png";
        pcl::io::saveRgbPNGFile(savePath, rgb_image, range_image.width, range_image.height);
        std::cout << "Saved ranges to: " + savePath + ". Size: " << range_image.height << ", " << range_image.width << std::endl;

        // Small delay to allow saving the png files.
        usleep(pngSaveTime);

        //Save the suppressed/inflated range image
        range_image.createFromPointCloud(point_cloud_suppress, angular_resolution_x, angular_resolution_y,
            pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
            scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
        ranges = range_image.getRangesArray();
        rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);
        savePath = basePath + savePath2 + fileNamePrefix + ".png";
        pcl::io::saveRgbPNGFile(savePath, rgb_image, range_image.width, range_image.height);
        std::cout << "Saved ranges to: " + savePath + ". Size: " << range_image.height << ", " << range_image.width << std::endl;

        usleep(pngSaveTime);


        // Open 3D viewer and add point cloud
        if (bOpenVisualizer)
        {
            pcl::visualization::PCLVisualizer viewer("3D Viewer");
            viewer.setBackgroundColor(1, 1, 1);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange > range_image_color_handler(range_image_ptr, 0, 0, 0);
            viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
            //viewer.addCoordinateSystem (1.0f, "global");
            //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
            //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
            viewer.initCameraParameters();
            setViewerPose(viewer, range_image.getTransformationToWorldSystem());

            // Range image window
            pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
            range_image_widget.showRangeImage(range_image);

            // Viewer window loop
            while (!viewer.wasStopped())
            {
                range_image_widget.spinOnce();
                viewer.spinOnce();
                pcl_sleep(0.01);

                if (live_update)
                {
                    scene_sensor_pose = viewer.getViewerPose();
                    range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
                        pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                        scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
                    range_image_widget.showRangeImage(range_image);
                }
            }
        }

    }   // end for loop for all files

}   // end main