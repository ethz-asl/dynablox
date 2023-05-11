# Check target dir.
if [ -z $1 ]; 
then 
  echo "No target directory specified.";
  echo "Usage: ./download_dynablox_data.sh <target_dir>";
  exit 1
else 
  echo "Download the Dynablox dataset to '$1'?"; 
  read -p "[Y/N]? " -n 1 -r
  if [[ ! $REPLY =~ ^[Yy]$ ]]
  then
    echo "Exiting."; 
    exit 1
  fi
fi

# Download.
mkdir -p $1
echo "";
echo "Downloading and processing scenes. This may take a few minutes ...";

# Get data.
gdown 1RDmv3YabIRfx6i0qsr5aFWozVnwJyFun -O $1
unzip $1/processed.zip -d $1
rm $1/processed.zip

# Finished.
echo "Successfully downloaded and processed the Dynablox dataset to '$1'.";
exit 0