# Check target dir.
if [ -z $1 ]; 
then 
  echo "No target directory specified.";
  echo "Usage: ./download_doals.sh <target_dir>";
  exit 1
else 
  echo "Download the DOALS dataset to '$1'?"; 
  read -p "[Y/N]? " -n 1 -r
  if [[ ! $REPLY =~ ^[Yy]$ ]]
  then
    echo "Exiting."; 
    exit 1
  fi
fi

# Download.
mkdir -p $1
echo ""
echo "Downloading and processing scenes. This may take a few minutes ...";
wget http://robotics.ethz.ch/~asl-datasets/2021_ICRA_dynamic_object_lidar_dataset/scenes/hauptgebaeude.zip -P $1
unzip $1/hauptgebaeude.zip -d $1
rm $1/hauptgebaeude.zip

# Finished.
echo "Successfully downloaded and processed the DOALS dataset to '$1'.";
exit 0