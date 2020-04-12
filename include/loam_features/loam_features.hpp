#ifndef LOAM_FEATURES_HPP_
#define LOAM_FEATURES_HPP_

//C++
#include <iostream>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr pclConstPtr;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pclPtr;
typedef std::pair<size_t, size_t> IndexRange;

/** Point label options. */
enum PointLabel
{
    CORNER_SHARP = 2,      ///< sharp corner point
    CORNER_LESS_SHARP = 1, ///< less sharp corner point
    SURFACE_LESS_FLAT = 0, ///< less flat surface point
    SURFACE_FLAT = -1      ///< flat surface point
};

class MultiScanMapper
{
public:
    /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
    MultiScanMapper(const float &lowerBound = -15,
                    const float &upperBound = 15,
                    const uint16_t &nScanRings = 16);

    const float &getLowerBound() { return _lowerBound; }
    const float &getUpperBound() { return _upperBound; }
    const uint16_t &getNumberOfScanRings() { return _nScanRings; }

    /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
    void set(const float &lowerBound,
             const float &upperBound,
             const uint16_t &nScanRings);

    /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
    int getRingForAngle(const float &angle);

    /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
    static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

    /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
    static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

    /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
    static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };

    /** Multi scan mapper for Ouster O1-64 according to data sheet. */
    static inline MultiScanMapper Ouster_O1_64() { return MultiScanMapper(-16.611f, 16.611f, 64); };

    /** Multi scan mapper for Ouster O1-16 according to data sheet. */
    static inline MultiScanMapper Ouster_O1_16() { return MultiScanMapper(-16.611f, 16.611f, 16); };

private:
    float _lowerBound;    ///< the vertical angle of the first scan ring
    float _upperBound;    ///< the vertical angle of the last scan ring
    uint16_t _nScanRings; ///< number of scan rings
    float _factor;        ///< linear interpolation factor
};

struct Configuration
{
    const float scanPeriod = 0.1;
    const int curvatureRegion = 5;
    const int nFeatureRegions = 6;
    const int maxCornerSharp = 2;
    const int maxCornerLessSharp = 10 * maxCornerSharp;
    const int maxSurfaceFlat = 4;
    const float surfaceCurvatureThreshold = 0.1;
    const float lessFlatFilterSize = 0.2;
};

class LoamFeatures
{
private:
    ros::Subscriber pclSub_;
    ros::Publisher pclFeaturesPub_;
    MultiScanMapper _scanMapper; ///< mapper for mapping vertical point angles to scan ring IDs
    std::vector<pcl::PointCloud<pcl::PointXYZI>> _laserCloudScans;
    pcl::PointCloud<pcl::PointXYZI> _laserCloud; ///< full resolution input cloud //smk:contains all points but now ring-wise sorted
    std::vector<IndexRange> _scanIndices;        ///< start and end indices of the individual scans withing the full resolution cloud
    Configuration _config;
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;     ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp; ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat;     ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat; ///< less flat surface points cloud
    std::vector<float> _regionCurvature;                    ///< point curvature buffer
    std::vector<PointLabel> _regionLabel;                   ///< point label buffer
    std::vector<size_t> _regionSortIndices;                 ///< sorted region indices based on point curvature
    std::vector<int> _scanNeighborPicked;                   ///< flag if neighboring point was already picked

public:
    LoamFeatures(ros::NodeHandle &nh, ros::NodeHandle &nhp);
    ~LoamFeatures(){};
    void pclCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &ros_pcl_ptr);
    void arrangePCLInScanLines(const pcl::PointCloud<pcl::PointXYZI> &laserCloudIn, float scanPeriod);
    void extractFeatures(const uint16_t &beginIdx = 0);
    void setRegionBuffersFor(const size_t &startIdx, const size_t &endIdx);
    void setScanBuffersFor(const size_t &startIdx, const size_t &endIdx);
    void markAsPicked(const size_t &cloudIdx, const size_t &scanIdx);

    // Calculate the squared difference of the given two points.
    template <typename PointT>
    inline float calcSquaredDiff(const PointT &a, const PointT &b)
    {
        float diffX = a.x - b.x;
        float diffY = a.y - b.y;
        float diffZ = a.z - b.z;

        return diffX * diffX + diffY * diffY + diffZ * diffZ;
    }

    //Calculate the squared difference of the given two points with weighting
    template <typename PointT>
    inline float calcSquaredDiff(const PointT &a, const PointT &b, const float &wb)
    {
        float diffX = a.x - b.x * wb;
        float diffY = a.y - b.y * wb;
        float diffZ = a.z - b.z * wb;

        return diffX * diffX + diffY * diffY + diffZ * diffZ;
    }

    // Calculate the absolute distance of the point to the origin.
    template <typename PointT>
    inline float calcPointDistance(const PointT &p)
    {
        return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    // Calculate the squared distance of the point to the origin.
    template <typename PointT>
    inline float calcSquaredPointDistance(const PointT &p)
    {
        return p.x * p.x + p.y * p.y + p.z * p.z;
    }
};

#endif