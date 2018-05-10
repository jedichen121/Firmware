#!/bin/bash
if [[ ($1 == "-h") || ($1 == "--help") ]]
then
    echo "Usage:"
    echo "$0 [-h,-help] [-v,--vehicle] [-g,--gcs-ip-port] [-r,--radio-device] 
        -h, --help  show this help text
	-v, --vehicle vehicle name you need to have a corresponding config file
	-g, --gcs-ip-port use UDP connection to the ip address of groundcontrol station, e.g. 192.168.11:14550
	-r, --radio-device use radio connection. e.g. /dev/ttyUSB0"
    exit 0
fi


while [[ $# > 1 ]]
do
    key="$1"

    case $key in
	-v|--vehicle)
	    VEHICLE="$2"
	    shift
	    ;;
	
	-g|--gcs-ip-port)
	    GCS="$2"
	    shift
	    ;;
	-r|--radio-device)
	    RADIO="$2"
	    shift
	    ;;
	*) # unknown option
	    ;;
    esac
    shift
done

if [[($VEHICLE == '')]]
then
	echo 'Vehicle not specified'
	$0 -h
	exit 1
fi

available_vehicles=$(list-vehicles-px4.sh)

if [[($available_vehicles != *$VEHICLE*)]]
then
	echo "vehicle name $VEHICLE does not exists."
	echo "available vehicles:"
	list-vehicles-px4.sh
	exit 1
fi

tmp_full=$(mktemp /tmp/full_XXXX.config)
tmp_vehicle=$(mktemp /tmp/vehicle_XXXX.config)
tmp_gcs=$(mktemp /tmp/gcs_XXXX.config)
tmp_radio=$(mktemp /tmp/radio_XXXX.config)
config_path=${MAVkitPATH}/px4_config/
cat $config_path/vehicles/$VEHICLE.config > $tmp_vehicle

echo "config_path: $config_path"

if [[($GCS != '')]]
then
	ip="$(cut -d':' -f1 <<<"$GCS")"
	port="$(cut -d':' -f2 <<<"$GCS")"
	sed -- "s|IP|${ip}|g" $config_path/templates/template-gcs.config > $tmp_gcs
	sed -i -- "s|PORT|${port}|g" $tmp_gcs
fi

if [[($RADIO != '')]]
then
	echo "radio device is: " $RADIO
	sed -- "s|DEVICE|${RADIO}|g" $config_path/templates/template-radio.config > $tmp_radio
	cat $tmp_radio
fi

cat $tmp_vehicle $tmp_gcs $tmp_radio $config_path/templates/template-complete.config> $tmp_full
echo "***** constucted config file reads: *****"
cat $tmp_full

echo "tmp_full: $tmp_full"

fs_path=$MAVkitPATH/px4_vehicle_fs/$VEHICLE/
cd $fs_path
echo "fs_path: $fs_path"
echo "***** starting PX4: *****"
sudo $MAVkitPATH/bin/px4 $tmp_full
echo "MAVkitPATH: $MAVkitPATH“
