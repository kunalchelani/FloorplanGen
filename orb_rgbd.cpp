#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>


int main(int argc, char **argv)
{
	std::cout << "Started" << std::endl;

	if (argc != 8){
		std::cout << "Usage : ./orb_rgbd vocab_file settings_file RGBImagesDirectory DepthImagesDirectory AssociationFilename trajectoryFilename keyframeTrajectoryFilename" << std::endl;
		return 0;
	}
	
	vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    
    string strRGBImagesDirectory = string(argv[3]);
    string strDepthImagesDirectory = string(argv[4]);
    string strAssociationFilename = string(argv[5]);
    string trajectoryFilename = string(argv[6]);
    string keyframeTrajectoryFilename =  string(argv[7]);

    int num_files;
    
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());

    string s;
    getline(fAssociation, s);
    if(!s.empty()) {

    	stringstream ss;
    	ss << s;
    	ss >> num_files;
    }

    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string sRGB, sD;
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }

    std::cout << vstrImageFilenamesD.size() << std::endl;
    std::cout << vstrImageFilenamesRGB.size() << std::endl;

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    std::cout << endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << num_files << std::endl << std::endl;
    
    cv::Mat imRGB, imD;

    for(int ni=0; ni< num_files; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3]) +"/"+ vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[4]) +"/"+ vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = 1305031453.359684 + ni*0.4;

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);
/*
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
*/    
    }

    SLAM.Shutdown();

    std::cout << "Done" << std::endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(trajectoryFilename);
    SLAM.SaveKeyFrameTrajectoryTUM(keyframeTrajectoryFilename);

	return 0;


}