if [ $# -lt 1 ]; then
    echo "usage: pushrover.sh <username>@<ip address>"
    exit 1
fi

remote=$1
echo "pushing to $remote"
echo "tarring robocluster..."
pushd ../robocluster
tar -czf robocluster.tar.gz ./*
scp robocluster.tar.gz ${remote}:robocluster
popd
echo "tarring rover-processes..."
tar -czf rover-processes.tar.gz *.py libraries *.sh
echo "Pushing..."
scp  rover-processes.tar.gz ${remote}:rover-processes
echo "Unpacking files..."
ssh $remote "cd robocluster; tar -xzf robocluster.tar.gz; cd  ~/rover-processes; tar -xzf rover-processes.tar.gz"
echo "Done!"
