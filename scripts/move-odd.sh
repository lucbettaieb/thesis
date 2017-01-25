mkdir odds
for i in $(\ls | grep [13579].jpg)
do
mv $i odds/$i
done
