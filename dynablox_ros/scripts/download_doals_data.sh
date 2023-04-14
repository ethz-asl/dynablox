# Check target dir.
if [ -z $1 ]; 
then 
  echo "No target directory specified.";
  echo "Usage: ./download_doals_data.sh <target_dir>";
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

# Hauptgebaeude.
wget http://robotics.ethz.ch/~asl-datasets/2021_ICRA_dynamic_object_lidar_dataset/scenes/hauptgebaeude.zip -P $1
unzip $1/hauptgebaeude.zip -d $1
rm $1/hauptgebaeude.zip
mv $1/hauptgebaeude/sequence_1/2020-02-20-11-58-45.bag $1/hauptgebaeude/sequence_1/bag.bag
mv $1/hauptgebaeude/sequence_2/2020-02-20-12-04-48.bag $1/hauptgebaeude/sequence_2/bag.bag

# Niederdorf.
wget http://robotics.ethz.ch/~asl-datasets/2021_ICRA_dynamic_object_lidar_dataset/scenes/niederdorf.zip -P $1
unzip $1/niederdorf.zip -d $1
rm $1/niederdorf.zip
mv $1/niederdorf/sequence_1/2020-02-19-16-12-14.bag $1/niederdorf/sequence_1/bag.bag
mv $1/niederdorf/sequence_2/2020-02-19-16-20-17.bag $1/niederdorf/sequence_2/bag.bag

# Shopville.
wget http://robotics.ethz.ch/~asl-datasets/2021_ICRA_dynamic_object_lidar_dataset/scenes/shopville.zip -P $1
unzip $1/shopville.zip -d $1
rm $1/shopville.zip
mv $1/shopville/sequence_1/2020-02-20-16-39-51.bag $1/shopville/sequence_1/bag.bag
mv $1/shopville/sequence_2/2020-02-20-16-43-52.bag $1/shopville/sequence_2/bag.bag

# Station.
wget http://robotics.ethz.ch/~asl-datasets/2021_ICRA_dynamic_object_lidar_dataset/scenes/station.zip -P $1
unzip $1/station.zip -d $1
rm $1/station.zip
mv $1/station/sequence_1/2020-02-20-17-28-39.bag $1/station/sequence_1/bag.bag
mv $1/station/sequence_2/2020-02-20-17-35-27.bag $1/station/sequence_2/bag.bag

# Finished.
echo "Successfully downloaded and processed the DOALS dataset to '$1'.";
exit 0