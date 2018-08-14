# Usage ./runAll.sh <baseDirectory> <f for filtered depth files | anything else> <k for keyframes only| anything else> <nth frame to be used for reconstruction>

echo "Starting process. Writing Association File for ${1}"

baseDirectory=${1}

if [ ${2} == 'f' ]
then 
	filtered=true
else
	filtered=false
fi

if [ ${3} == 'k' ]
then 
	keyFramesOnly=true	
	keyOrFull="keyframes"
else
	keyFramesOnly=false
	keyOrFull="full"
fi

nthFrames=${4}

RGBImagesDirectory="${baseDirectory}/images/rgb/"
DepthImagesDirectory="${baseDirectory}/images/depth/"
FilteredDepthImagesDirectory="${baseDirectory}/images/depth_filtered"
AssociationFilename="${baseDirectory}/associations.txt"
trajectoryFilename="${baseDirectory}/trajectory.txt"
keyframeTrajectoryFilename="${baseDirectory}/keyframeTrajectory.txt"

echo $RGBImagesDirectory
echo $DepthImagesDirectory
echo $AssociationFilename
echo $trajectoryFilename
echo $keyframeTrajectoryFilename
echo $keyFramesOnly
echo $filtered

# Getting Camera Estimates

python ~/orb_rgbd/python/associate_ours.py ${1}
echo "Association file written. Starting to obtain trajectories."

if $filtered

then

	/home/cvlab/orb_rgbd/build/orb_rgbd /home/cvlab/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/cvlab/ORB_SLAM2/Examples/RGB-D/TUM1.yaml $RGBImagesDirectory $FilteredDepthImagesDirectory $AssociationFilename $trajectoryFilename $keyframeTrajectoryFilename

else

	/home/cvlab/orb_rgbd/build/orb_rgbd /home/cvlab/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/cvlab/ORB_SLAM2/Examples/RGB-D/TUM1.yaml $RGBImagesDirectory $DepthImagesDirectory $AssociationFilename $trajectoryFilename $keyframeTrajectoryFilename
fi

echo "Obtained trajectory. Starting with Reconstruction"

# Pointcloud Reconstruction

rgbList="${baseDirectory}/${keyOrFull}/rgb_list.txt"
depthList="${baseDirectory}/${keyOrFull}/depth_list.txt"
outputCloudName="${baseDirectory}/${keyOrFull}/out_cloud.ply"
stripNum=${5}

if $filtered
then
	python ~/orb_rgbd/python/assocToIndiLists.py $baseDirectory 1 ${keyOrFull}
else
	python ~/orb_rgbd/python/assocToIndiLists.py $baseDirectory 0 ${keyOrFull}
fi

if $keyFramesOnly
then
	echo "Running Keyframes only"  
	python ~/orb_rgbd/Utilities/generate_registered_pointcloud.py ${rgbList} ${depthList} $keyframeTrajectoryFilename $outputCloudName --nth ${nthFrames}
else
	echo "Running full reconstruction"
	python ~/orb_rgbd/Utilities/generate_registered_pointcloud.py ${rgbList} ${depthList} $trajectoryFilename $outputCloudName --nth ${nthFrames}
fi

echo "Getting line strips and intersections"

# Getting strips and lines

# python ~/orb_rgbd/python/getStrips.py "${baseDirectory}/${keyOrFull}/out_cloud.ply" "${baseDirectory}/${keyOrFull}/strips/"

# /home/cvlab/orb_rgbd/build/lines "${baseDirectory}/${keyOrFull}/strips/strip{stripNum}.ply" "${baseDirectory}/${keyOrFull}/strips/"
