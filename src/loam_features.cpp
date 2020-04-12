#include "loam_features.hpp"

//Constructor
LoamFeatures::LoamFeatures(ros::NodeHandle &nh, ros::NodeHandle &nhp)
{

    _scanMapper = MultiScanMapper::Velodyne_VLP_16();

    //Subscriber
    pclSub_ = nh.subscribe("/pcl", 2, &LoamFeatures::pclCallback, this);

    //Publsiher
    pclFeaturesPub_ = nhp.advertise<sensor_msgs::PointCloud2>("loam_features", 10);
}

//Callback
void LoamFeatures::pclCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &pcl_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr inPCL(new pcl::PointCloud<pcl::PointXYZI>);
    inPCL = boost::make_shared<std::remove_const<std::remove_reference<decltype(*pcl_ptr)>::type>::type>(*pcl_ptr);

    //arrange
    std::clock_t arrangeScanLines_t = std::clock();
    arrangePCLInScanLines(*inPCL, _config.scanPeriod);
    std::cout << "Timing - Arrange Scan Lines: " << float(std::clock() - arrangeScanLines_t) / CLOCKS_PER_SEC * 1000.0 << " ms\n";

    //extract features
    std::clock_t extractFeatures_t = std::clock();
    extractFeatures();
    std::cout << "Timing - Extract Features: " << float(std::clock() - extractFeatures_t) / CLOCKS_PER_SEC * 1000.0 << " ms\n";

    //Publish Feature Cloud
    if (pclFeaturesPub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI> pubCloud;
        pubCloud += _cornerPointsSharp;
        pubCloud += _cornerPointsLessSharp;
        pubCloud += _surfacePointsFlat;
        pubCloud += _surfacePointsLessFlat;
        pubCloud.header = pcl_ptr->header;
        pubCloud.header.frame_id = "/loam";
        pclFeaturesPub_.publish(pubCloud);
    }
}

//Arrange the pointcloud in Scanlines
void LoamFeatures::arrangePCLInScanLines(const pcl::PointCloud<pcl::PointXYZI> &laserCloudIn, float scanPeriod)
{
    size_t cloudSize = laserCloudIn.size();

    // determine scan start and end orientations
    float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
    float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y, laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    pcl::PointXYZI point;
    _laserCloudScans.clear();
    _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
    // clear all scanline points
    // std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto &&v) { v.clear(); });

    // extract valid points from input cloud
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn[i].y;
        point.y = laserCloudIn[i].z;
        point.z = laserCloudIn[i].x;

        // skip NaN and INF valued points
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }

        // skip zero valued points
        if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) //smk:why not just check z only?
        {
            continue;
        }

        // calculate vertical point angle and scan ID
        float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
        int scanID = _scanMapper.getRingForAngle(angle);
        if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0)
        {
            continue;
        }

        // calculate horizontal point angle
        float ori = -std::atan2(point.x, point.z);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;

            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        // calculate relative scan time based on point orientation
        float relTime = scanPeriod * (ori - startOri) / (endOri - startOri); //smk: given the orientation of the current point and knowning the starting,ending orientations and total scan time we can find the relative time at which this point was scanned when the lidar was doing its sweep or scan.
        point.intensity = scanID + relTime;
        _laserCloudScans[scanID].push_back(point);
    }

    // construct sorted full resolution cloud
    _laserCloud.clear();
    _scanIndices.clear();
    size_t cloudSizeScans = 0;
    for (int i = 0; i < _laserCloudScans.size(); i++)
    {
        _laserCloud += _laserCloudScans[i];

        IndexRange range(cloudSizeScans, 0);
        cloudSizeScans += _laserCloudScans[i].size();
        range.second = cloudSizeScans > 0 ? cloudSizeScans - 1 : 0;
        _scanIndices.push_back(range);
    }
}

//Extract Feature Clouds from Scan Lines
void LoamFeatures::extractFeatures(const uint16_t &beginIdx)
{

    //Clear out old features
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();
    _regionCurvature.clear();
    _regionLabel.clear();
    _regionSortIndices.clear();
    _scanNeighborPicked.clear();

    // extract features from individual scans
    size_t nScans = _scanIndices.size();
    for (size_t i = beginIdx; i < nScans; i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
        size_t scanStartIdx = _scanIndices[i].first;
        size_t scanEndIdx = _scanIndices[i].second;

        // skip empty scans
        if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion)
        {
            continue;
        }

        // reset scan buffers
        setScanBuffersFor(scanStartIdx, scanEndIdx);

        // extract features from equally sized scan regions
        for (int j = 0; j < _config.nFeatureRegions; j++)
        {
            size_t sp = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - j) + (scanEndIdx - _config.curvatureRegion) * j) / _config.nFeatureRegions;
            size_t ep = ((scanStartIdx + _config.curvatureRegion) * (_config.nFeatureRegions - 1 - j) + (scanEndIdx - _config.curvatureRegion) * (j + 1)) / _config.nFeatureRegions - 1;

            // skip empty regions
            if (ep <= sp)
            {
                continue;
            }

            size_t regionSize = ep - sp + 1;

            // reset region buffers //smk: this calculates the curvature for points in the scans
            setRegionBuffersFor(sp, ep);

            // extract corner features
            int largestPickedNum = 0;
            for (size_t k = regionSize; k > 0 && largestPickedNum < _config.maxCornerLessSharp;) //smk: maxCornerLessSharp = 10*maxCornerSharp
            {
                size_t idx = _regionSortIndices[--k];
                size_t scanIdx = idx - scanStartIdx;
                size_t regionIdx = idx - sp;

                //smk: pick sharp points, the sharp and less sharp need to be above "surfaceCurvatureThreshold", technically there is no difference in btw the two only some of them are marked are sharp to reduce data for laser odometry
                if (_scanNeighborPicked[scanIdx] == 0 && _regionCurvature[regionIdx] > _config.surfaceCurvatureThreshold)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= _config.maxCornerSharp)
                    {
                        _regionLabel[regionIdx] = CORNER_SHARP;
                        _cornerPointsSharp.push_back(_laserCloud[idx]);
                    }
                    else
                    {
                        _regionLabel[regionIdx] = CORNER_LESS_SHARP;
                    }
                    _cornerPointsLessSharp.push_back(_laserCloud[idx]);

                    markAsPicked(idx, scanIdx);
                }
            }

            // extract flat surface features //smk: points that are less than "surfaceCurvatureThreshold" and a certain number of them are marked as FLAT(according to per region 4 features), rest are marked as less flat
            int smallestPickedNum = 0;
            for (int k = 0; k < regionSize && smallestPickedNum < _config.maxSurfaceFlat; k++)
            {
                size_t idx = _regionSortIndices[k];
                size_t scanIdx = idx - scanStartIdx;
                size_t regionIdx = idx - sp;

                if (_scanNeighborPicked[scanIdx] == 0 && _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold)
                {
                    smallestPickedNum++;
                    _regionLabel[regionIdx] = SURFACE_FLAT;
                    _surfacePointsFlat.push_back(_laserCloud[idx]);

                    markAsPicked(idx, scanIdx);
                }
            }

            // extract less flat surface features
            for (int k = 0; k < regionSize; k++)
            {
                if (_regionLabel[k] <= SURFACE_LESS_FLAT)
                {
                    surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
                }
            }
        }

        // down size less flat surface point cloud of current scan
        pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(_config.lessFlatFilterSize, _config.lessFlatFilterSize, _config.lessFlatFilterSize);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        _surfacePointsLessFlat += surfPointsLessFlatScanDS;
    }
}

void LoamFeatures::setRegionBuffersFor(const size_t &startIdx, const size_t &endIdx)
{
    // resize buffers
    size_t regionSize = endIdx - startIdx + 1;
    _regionCurvature.resize(regionSize);
    _regionSortIndices.resize(regionSize);
    _regionLabel.assign(regionSize, SURFACE_LESS_FLAT); //smk:every point as a SURFACE_LESS_FLAT initially

    // calculate point curvatures and reset sort indices //smk: looping through # number of neighbor points(curvatureRegion) and calculate squared difference
    float pointWeight = -2 * _config.curvatureRegion;

    for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++)
    {
        float diffX = pointWeight * _laserCloud[i].x;
        float diffY = pointWeight * _laserCloud[i].y;
        float diffZ = pointWeight * _laserCloud[i].z;

        for (int j = 1; j <= _config.curvatureRegion; j++)
        {
            diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
            diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
            diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
        }

        _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        _regionSortIndices[regionIdx] = i;
    }

    // sort point curvatures
    for (size_t i = 1; i < regionSize; i++)
    {
        for (size_t j = i; j >= 1; j--)
        {
            if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx])
            {
                std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
            }
        }
    }
}

void LoamFeatures::setScanBuffersFor(const size_t &startIdx, const size_t &endIdx)
{
    // resize buffers
    size_t scanSize = endIdx - startIdx + 1;
    _scanNeighborPicked.assign(scanSize, 0);

    // mark unreliable points as picked -- smk: points that have depth difference greater than 0.1 meters(?) are set as already picked so they can be ignored later
    for (size_t i = startIdx + _config.curvatureRegion; i < endIdx - _config.curvatureRegion; i++)
    {
        const pcl::PointXYZI &previousPoint = (_laserCloud[i - 1]);
        const pcl::PointXYZI &point = (_laserCloud[i]);
        const pcl::PointXYZI &nextPoint = (_laserCloud[i + 1]);

        float diffNext = calcSquaredDiff(nextPoint, point);

        if (diffNext > 0.1)
        {
            float depth1 = calcPointDistance(point);
            float depth2 = calcPointDistance(nextPoint);

            if (depth1 > depth2)
            {
                float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

                if (weighted_distance < 0.1)
                {
                    std::fill_n(&_scanNeighborPicked[i - startIdx - _config.curvatureRegion], _config.curvatureRegion + 1, 1);

                    continue;
                }
            }
            else
            {
                float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

                if (weighted_distance < 0.1)
                {
                    std::fill_n(&_scanNeighborPicked[i - startIdx + 1], _config.curvatureRegion + 1, 1);
                }
            }
        }

        float diffPrevious = calcSquaredDiff(point, previousPoint);
        float dis = calcSquaredPointDistance(point);

        if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) //smk:this will reject a point if the difference in distance or depth of point and its neighbors is outside a bound i.e. rejecting very sharp ramp like points
        {
            _scanNeighborPicked[i - startIdx] = 1;
        }
    }
}

void LoamFeatures::markAsPicked(const size_t &cloudIdx, const size_t &scanIdx)
{
    _scanNeighborPicked[scanIdx] = 1;

    //smk: not only mark the point as picked but mark all the points used in the curvatureRegion around it as picked as well
    for (int i = 1; i <= _config.curvatureRegion; i++)
    {
        if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05)
        {
            break;
        }

        _scanNeighborPicked[scanIdx + i] = 1;
    }

    for (int i = 1; i <= _config.curvatureRegion; i++)
    {
        if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05)
        {
            break;
        }

        _scanNeighborPicked[scanIdx - i] = 1;
    }
}

MultiScanMapper::MultiScanMapper(const float &lowerBound,
                                 const float &upperBound,
                                 const uint16_t &nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound)) //smk:for VLP16 0.5
{
}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
    _lowerBound = lowerBound;
    _upperBound = upperBound;
    _nScanRings = nScanRings;
    _factor = (nScanRings - 1) / (upperBound - lowerBound);
}

int MultiScanMapper::getRingForAngle(const float &angle)
{
    return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}