rsync -av --exclude="*.o" --exclude="robotproject" . debian@192.168.7.2:robotproject
ssh debian@192.168.7.2 "cd robotproject;make"