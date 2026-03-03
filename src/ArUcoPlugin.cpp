#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <algorithm>

using namespace cv;

static std::shared_ptr<aruco::ArucoDetector> g_detector;
static std::vector<int> g_ids;
static std::vector<std::vector<Point2f>> g_corners;

//camera intrinsics
static Mat g_cameraMatrix;
static Mat g_distCoeffs;
static bool g_hasIntrinsics;

static std::vector<int> g_whitelist;
static bool g_useWhitelist = false;

static bool inited = false;

extern "C" {
// ---------------------------------------------------------------------
// Init detector
// ---------------------------------------------------------------------
void ArucoInit(int dictionaryId)
{
    auto dict = aruco::getPredefinedDictionary(
        aruco::PredefinedDictionaryType(dictionaryId));

    aruco::DetectorParameters params;

    g_detector = std::make_shared<aruco::ArucoDetector>(dict, params);
}

// ---------------------------------------------------------------------
// ArucoSetWhitelist
// ---------------------------------------------------------------------
void ArucoSetWhitelist(int* markerIds, int count) 
{
    g_whitelist.clear();
    g_useWhitelist = (count > 0);

    for (int i = 0; i < count; i++) {
        g_whitelist.push_back(markerIds[i]);
    }
}

// ---------------------------------------------------------------------
// Camera intrinsics
// ---------------------------------------------------------------------
void ArucoSetCameraIntrinsics(
    float fx, float fy,
    float cx, float cy
)
{
    g_cameraMatrix = Mat::eye(3, 3, CV_64F);
    g_cameraMatrix.at<double>(0, 0) = fx;
    g_cameraMatrix.at<double>(1, 1) = fy;
    g_cameraMatrix.at<double>(0, 2) = cx;
    g_cameraMatrix.at<double>(1, 2) = cy;

    // AR Foundation provides undistorted images
    g_distCoeffs = Mat::zeros(5, 1, CV_64F);

    g_hasIntrinsics = true;
}

// ---------------------------------------------------------------------
// Process image (RGBA32)
// ---------------------------------------------------------------------
void ArucoProcess(uint8_t* rgba, int width, int height)
{
    Mat img(height, width, CV_8UC4, rgba);
    Mat gray;

    cvtColor(img, gray, COLOR_RGBA2GRAY);
    //match unity camera
    // cv::rotate(gray, gray, ROTATE_90_CLOCKWISE);
    // cv::flip(gray, gray, 1);
    cv::rotate(gray, gray, ROTATE_180);

    if (g_hasIntrinsics && !inited) {
        inited = true;
        g_cameraMatrix.at<double>(0, 2) = width - g_cameraMatrix.at<double>(0, 2);   // cx flip
        g_cameraMatrix.at<double>(1, 2) = height - g_cameraMatrix.at<double>(1, 2);  // cy flip
    }

    g_ids.clear();
    g_corners.clear();
    std::vector<std::vector<Point2f>> rejected;

    g_detector->detectMarkers(gray, g_corners, g_ids, rejected);
    
    // Refine corners
    if (!g_corners.empty())
    {
        cv::Size winSize(5, 5);  // окно поиска
        cv::Size zeroZone(-1, -1);
        cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 
            40,  // max iterations
            0.001  // epsilon
        );
        
        for (auto& corners : g_corners)
        {
            cv::cornerSubPix(gray, corners, winSize, zeroZone, criteria);
        }
    }

    //whitelist processing
    if (g_useWhitelist && !g_whitelist.empty()) {
        std::vector<int> filteredIds;
        std::vector<std::vector<Point2f>> filteredCorners;

        for (size_t i = 0; i < g_ids.size(); i++) {
            if (std::find(g_whitelist.begin(), g_whitelist.end(), g_ids[i]) != g_whitelist.end()) {
                filteredIds.push_back(g_ids[i]);
                filteredCorners.push_back(g_corners[i]);
            }
        }

        g_ids = filteredIds;
        g_corners = filteredCorners;
    }
}


// ---------------------------------------------------------------------
// Return number of detected markers
// ---------------------------------------------------------------------
int ArucoGetCount()
{
    return (int)g_ids.size();
}


// ---------------------------------------------------------------------
// Return detected marker ID by count
// ---------------------------------------------------------------------
int ArucoGetId(int index)
{
    if (index < 0 || index >= g_ids.size()) 
        return -1;
    return g_ids[index];
}

// ---------------------------------------------------------------------
// Return corners by ArUco ID
// ---------------------------------------------------------------------
int ArucoGetCornersByID(int arucoId, float* cornerX, float* cornerY) 
{
    for (size_t i = 0; i < g_ids.size(); i++) {
        if (g_ids[i] == arucoId) {
            for (int c = 0; c < 4; c++) {
                cornerX[c] = g_corners[i][c].x;
                cornerY[c] = g_corners[i][c].y;
            }
            return 4;
        }
    }

    return 0;
}

// ---------------------------------------------------------------------
// Pose Estimate by ArUco ID
// ---------------------------------------------------------------------
// TODO: Move marker size into init
bool ArucoEstimatePose(
    int arucoId, 
    float markerSize,
    float* rvec,
    float* tvec
)
{
    if (!g_hasIntrinsics) 
        return false;

    // Find marker by ArUco Id
    // TODO: maybe add mapping ArUcoID -> index
    int markerIndex = -1;
    for (size_t i = 0; i < g_ids.size(); i++) {
        if (g_ids[i] == arucoId) {
            markerIndex = (int)i;
            break;
        }
    }

    if (markerIndex < 0) 
        return false;

    // Define 3D object points for the marker corners
    // Marker coordinate system: center at (0,0,0), Z pointing up
    // Corners order: top-left, top-right, bottom-right, bottom-left
    std::vector<Point3f> objPoints;
    float half = markerSize / 2.0f;
    objPoints.push_back(Point3f(-half,  half, 0));
    objPoints.push_back(Point3f( half,  half, 0));
    objPoints.push_back(Point3f( half, -half, 0));
    objPoints.push_back(Point3f(-half, -half, 0));

    Mat rvecMat, tvecMat;
    
    // Use false,SOLVEPNP_IPPE_SQUARE ? 
    // Solve PnP to get pose
    // BASIC VARIANT
    bool success = cv::solvePnP(
        objPoints, 
        g_corners[markerIndex], 
        g_cameraMatrix, 
        g_distCoeffs, 
        rvecMat, 
        tvecMat
    );

    // ITERATIVE VARIANT
    // bool success = cv::solvePnP(
    //     objPoints, 
    //     g_corners[markerIndex], 
    //     g_cameraMatrix, 
    //     g_distCoeffs, 
    //     rvecMat, 
    //     tvecMat,
    //     false,
    //     cv::SOLVEPNP_IPPE_SQUARE
    // );

    // if (success) {
    //     cv::solvePnP(
    //         objPoints,
    //         g_corners[markerIndex],
    //         g_cameraMatrix,
    //         g_distCoeffs,
    //         rvecMat, 
    //         tvecMat,
    //         true,
    //         cv::SOLVEPNP_ITERATIVE
    //     );
    // }

    if (success)
    {
        // Copy results to output arrays
        rvec[0] = (float)rvecMat.at<double>(0);
        rvec[1] = (float)rvecMat.at<double>(1);
        rvec[2] = (float)rvecMat.at<double>(2);
        
        tvec[0] = (float)tvecMat.at<double>(0);
        tvec[1] = (float)tvecMat.at<double>(1);
        tvec[2] = (float)tvecMat.at<double>(2);
    }

    return success;
}

} // extern "C"
