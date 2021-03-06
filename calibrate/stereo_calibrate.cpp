/* *************** License:**************************
   Created by Ranik Guidolini  
   Based on the book:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     
   Given a list of chessboard images and the number of corners (nx, ny)
   on the chessboards, saves the instrinsics and extrinsics parameter
   ************************************************** */

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sstream>

using namespace std;

static void
StereoCalib(const char* imageList, int nx, int ny, float _squareSize, int saveCorners, int savePair, int saveRectified)
{
    char nome[150];
    bool isVerticalStereo = false;//OpenCV can handle left-right or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = _squareSize; //Chessboard square size in cm
    FILE* f = fopen(imageList, "rt");
    int i, j, lr, nframes, n = nx*ny, N = 0;
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {0,0};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    double Q[4][4];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    CvMat _Q = cvMat(4,4, CV_64F, Q);
    char buf[1024];
    int count=0, result=0;
    
// READ IN THE LIST OF CHESSBOARDS:
    if (!f)
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }
    
    for(i=0;;i++)
    {
        lr = i % 2;
        vector<CvPoint2D32f>& pts = points[lr];
        if (!fgets( buf, sizeof(buf)-3, f))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        IplImage* img = cvLoadImage( buf, 0 );
        if (!img)
            break;
        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);

//FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny), &temp[0], &count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
            
	    if (!result)   //Ranik
		    printf("\n%s\n", buf);
	    
	    if (timg != img)
                cvReleaseImage( &timg );

            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
                {
                    temp[j].x /= s;
                    temp[j].y /= s;
                }
            if( result )
                break;
        }
        if (saveCorners)
        {
            //printf("%s\n", buf);
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor ( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners (cimg, cvSize(nx, ny), &temp[0], count, result);

	    /*Save the image corners cordinates to file _ Edited by Ranik	
	    ofstream coordinates;
	    sprintf(nome, "pointCor%d.txt", i);
	    coordinates.open(nome);
	    coordinates << buf << endl;
	    for (int i = 0; i <= temp.size(); i++)
		{	
			coordinates << temp[i].x << endl;
			coordinates << temp[i].y << endl;
		}
		coordinates.close();//Edited by Ranik
	    */
	    //Save image in file instead of show it in a window
	    sprintf(nome, "cor%d.jpeg", i);
	    cvSaveImage(nome, cimg);
	    cvReleaseImage( &cimg );
	    //End of edition
        }
        else
            putchar('.');
	
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);
    	
	//assert( result != 0 );
        if (result)
        {
         //Calibration will suffer without subpixel interpolation
		cvFindCornerSubPix( img, &temp[0], count,
		cvSize(11, 11), cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01) );
		copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage (&img);
    }
    fclose(f);
    printf("\n");
    
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    nframes = active[0].size();//Number of good chessboads found
    objectPoints.resize(nframes*n);
    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
        objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), (objectPoints.begin() + n), (objectPoints.begin() + i*n) );
    npoints.resize(nframes,n);
    N = nframes*n;
    
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);

// CALIBRATE THE STEREO CAMERAS
    printf("Running stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
        &_imagePoints2, &_npoints,
        &_M1, &_D1, &_M2, &_D2,
        imageSize, &_R, &_T, &_E, &_F,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_ASPECT_RATIO +
        CV_CALIB_ZERO_TANGENT_DIST +
        CV_CALIB_SAME_FOCAL_LENGTH );
    printf(" done!\n");
// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
//Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_M2, &_D2, 0, &_M2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n));
    
    //Save the subpixel error to file ranik
    ofstream erro;
    erro.open("error.txt");
    erro << "avg err = " << avgErr/(nframes*n) << endl;
    erro.close();
    
//COMPUTE AND DISPLAY RECTIFICATION
        CvMat* mx1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* img1r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* img2r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* disp = cvCreateMat( imageSize.height,
            imageSize.width, CV_16S );
        CvMat* vdisp = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* pair;
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
        // IF BY CALIBRATED (BOUGUET'S METHOD)
        
	CvMat _P1 = cvMat(3, 4, CV_64F, P1);
	CvMat _P2 = cvMat(3, 4, CV_64F, P2);
	cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
	    &_R, &_T,
	    &_R1, &_R2, &_P1, &_P2, &_Q,
	    0/*CV_CALIB_ZERO_DISPARITY*/ );
	isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
	//Precompute maps for cvRemap()
	cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
	cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
	
	//Save parameters
	cvSave("M1.xml",&_M1);
	cvSave("D1.xml",&_D1);
	cvSave("R1.xml",&_R1);
	cvSave("P1.xml",&_P1);
	cvSave("M2.xml",&_M2);
	cvSave("D2.xml",&_D2);
	cvSave("R2.xml",&_R2);
	cvSave("P2.xml",&_P2);
	cvSave("Q.xml",&_Q);
	cvSave("R.xml",&_R);  //Ranik
	cvSave("T.xml",&_T);  //Ranik

	if (savePair || saveRectified)
	{
	    // RECTIFY THE IMAGES AND FIND DISPARITY MAPS
	    if( !isVerticalStereo )
		pair = cvCreateMat( imageSize.height, imageSize.width*2, CV_8UC3 );
	    else
		pair = cvCreateMat( imageSize.height*2, imageSize.width, CV_8UC3 );
	    
	    //Setup for finding stereo corrrespondences
	    /*CvStereoBMState *BMState = cvCreateStereoBMState();
	    assert(BMState != 0);
	    BMState->preFilterSize=41;
	    BMState->preFilterCap=31;
	    BMState->SADWindowSize=41;
	    BMState->minDisparity=-64;
	    BMState->numberOfDisparities=128;
	    BMState->textureThreshold=10;
	    BMState->uniquenessRatio=15;
	    */
	    for( i = 0; i < nframes; i++ )
	    {
		IplImage* img1=cvLoadImage(imageNames[0][i].c_str(),0);
		IplImage* img2=cvLoadImage(imageNames[1][i].c_str(),0);
		if( img1 && img2 )
		{
		    CvMat part;
		    cvRemap( img1, img1r, mx1, my1 );
		    cvRemap( img2, img2r, mx2, my2 );
		    /*if( !isVerticalStereo || useUncalibrated != 0 )
		    {
		      // When the stereo camera is oriented vertically,
		      // useUncalibrated==0 does not transpose the
		      // image, so the epipolar lines in the rectified
		      // images are vertical. Stereo correspondence
		      // function does not support such a case.
			//cvFindStereoCorrespondenceBM( img1r, img2r, disp,
			    BMState);
			cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
			//cvNamedWindow( "disparity" );
			//cvShowImage( "disparity", vdisp );
			//sprintf(nome, "P_disparity%d.jpeg", i);
			//sprintf(nome, "disparity%d.jpeg", i);
			//cvSaveImage(nome, vdisp);
			*/
		    if (saveRectified)
		    {
			sprintf(nome, "recL%d.png", i);
			cvSaveImage(nome, img1r);
			sprintf(nome, "recR%d.png", i);
			cvSaveImage(nome, img2r);
		    }
		    if (savePair)
		    {
			if( !isVerticalStereo )
			{
			    cvGetCols( pair, &part, 0, imageSize.width );
			    cvCvtColor( img1r, &part, CV_GRAY2BGR );
			    cvGetCols( pair, &part, imageSize.width, imageSize.width*2 );
			    cvCvtColor( img2r, &part, CV_GRAY2BGR );
			    for( j = 0; j < imageSize.height; j += 16 )
				cvLine( pair, cvPoint(0,j),
				cvPoint(imageSize.width*2,j),
				CV_RGB(0,255,0));
			}
			else
			{
			    cvGetRows( pair, &part, 0, imageSize.height );
			    cvCvtColor( img1r, &part, CV_GRAY2BGR );
			    cvGetRows( pair, &part, imageSize.height, imageSize.height*2 );
			    cvCvtColor( img2r, &part, CV_GRAY2BGR );
			    for( j = 0; j < imageSize.width; j += 16 )
				cvLine( pair, cvPoint(j,0),
				cvPoint(j,imageSize.height*2),
				CV_RGB(0,255,0));
			}
			sprintf(nome, "par%d.jpeg", i);
			cvSaveImage(nome, pair);;
		    }
		}
		    cvReleaseImage( &img1 );
		    cvReleaseImage( &img2 );
		}
	    }
	    //cvReleaseStereoBMState(&BMState);
        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );
        cvReleaseMat( &img1r );
        cvReleaseMat( &img2r );
        cvReleaseMat( &disp );
}
int
main(int argc, char *argv[])
{
    int nx, ny, saveCorners=0, savePair=0, saveRectified=0;
    float squareSize;
    int fail = 0;
    
    //Check command line
    if (argc < 5 || argc > 8)
    {
        fprintf(stderr,"USAGE: %s imageList nx ny squareSize -sc -sp\n",argv[0]);
        fprintf(stderr,"\t imageList : Filename of the image list (string). Example : list.txt\n");
        fprintf(stderr,"\t nx : Number of horizontal squares (int > 0). Example : 9\n");
        fprintf(stderr,"\t ny : Number of vertical squares (int > 0). Example : 6\n");
        fprintf(stderr,"\t squareSize : Size of a square (float > 0) in cm. Example : 2.5\n");
	fprintf(stderr,"\t -sc : Optional, if added it will save the chessboard images with the corners marked\n");
	fprintf(stderr,"\t -sp : Optional, if added it will save the pairs of rectified images with epipolar lines\n");
	fprintf(stderr,"\t -sr : Optional, if added it will save the rectified images\n");
        return 1;
    } 

    nx = atoi(argv[2]);
    ny = atoi(argv[3]);
    squareSize = (float)atof(argv[4]);

    if (nx <= 0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: nx value can not be <= 0\n");
    }
    if (ny <= 0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: ny value can not be <= 0\n");
    }   
    if (squareSize <= 0.0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: squareSize value can not be <= 0\n");
    }
    for (int i = 5; i < argc; i++)
    {
	if (!strcmp(argv[i], "-sc"))
	    saveCorners = 1;
	if (!strcmp(argv[i], "-sp"))
	    savePair = 1;
	if (!strcmp(argv[i], "-sr"))
	    saveRectified = 1;
    }

    if(fail != 0) return 1;

    StereoCalib(argv[1], nx, ny, squareSize, saveCorners, savePair, saveRectified);
    return 0;
}