#/!bin/sh
AR_POSE_DIR=`rospack find ar_pose`
if [ $? -ne 0 ]; then
  echo "Could not find package 'ar_pose'"
  exit -1
fi

OUTDIR=$AR_POSE_DIR/data
if [ ! -d $OUTDIR ]; then
  echo "Package 'ar_pose' ($AR_POSE_DIR) has no subfolder data"
  exit -1
fi

SCRIPTPATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
NFILES=`ls $SCRIPTPATH/*.patt -1 | wc -l`
if [ $NFILES -eq 0 ]; then
  echo "No pattern file found in $SCRIPTPATH"
  exit 0
fi

echo "Copying $NFILES pattern files to $OUTDIR..."
cp $SCRIPTPATH/*.patt $OUTDIR
echo "Done"
ls $OUTDIR -al
exit 0
