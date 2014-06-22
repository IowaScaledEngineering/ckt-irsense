#-------------------------------------------------------
# Iowa Scaled Engineering
#-------------------------------------------------------

Element[0x00000000 "IRSENSE-CONNECT-2SIDE" "" "" 0 0 0 0 0 100 0x00000000]
(
	# West
	Pad[-2992 -7500 -4000 -7500 2835 2000 3435 "1" "1" "square,nopaste"]
	Pad[-2992 -2500 -4000 -2500 2835 2000 3435 "2" "2" "square,nopaste"]
	Pad[-2992 2500 -4000 2500 2835 2000 3435 "3" "3" "square,nopaste"]
	Pad[-2992 7500 -4000 7500 2835 2000 3435 "4" "4" "square,nopaste"]
	# East
	Pad[-2992 7500 -4000 7500 2835 2000 3435 "5" "5" "square,onsolder,nopaste"]
	Pad[-2992 2500 -4000 2500 2835 2000 3435 "6" "6" "square,onsolder,nopaste"]
	Pad[-2992 -2500 -4000 -2500 2835 2000 3435 "7" "7" "square,onsolder,nopaste"]
	Pad[-2992 -7500 -4000 -7500 2835 2000 3435 "8" "8" "square,onsolder,nopaste"]
	# Outline
	ElementArc[-9000 -9768 500 500 0 360 1000]
)
