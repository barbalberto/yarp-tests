# run using standard port

if [ ! $1 ] 
	then
	echo "missing branch name"
	exit
fi


function launch {
	reset
  	echo "$@"
  	eval $@
}


launch "./envelopeConsumer --branch $1 --carrier udp"
launch #./envelopeConsumer --branch $1 --carrier udp
launch ./envelopeConsumer --branch $1 --carrier tcp
launch ./envelopeConsumer --branch $1 --carrier tcp
launch #./envelopeConsumer --branch $1 --carrier tcp
launch ./envelopeConsumer --branch $1 --carrier fast_tcp
launch ./envelopeConsumer --branch $1 --carrier fast_tcp
launch #./envelopeConsumer --branch $1 --carrier fast_tcp

launch # run using buffered port

launch ./envelopeConsumer --buff --branch $1 --carrier udp
launch ./envelopeConsumer --buff --branch $1 --carrier tcp
launch ./envelopeConsumer --buff --branch $1 --carrier fast_tcp

