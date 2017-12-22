
function launch {
	reset
  	echo "$@"
  	eval $@
}


# run using standard port

launch ./envelopeProducer --size 10   --run 1000
launch ./envelopeProducer --size 100  --run 1000
launch #./envelopeProducer --size 1000 --run 1000
launch ./envelopeProducer --size 10   --run 1000
launch ./envelopeProducer --size 100  --run 1000
launch #./envelopeProducer --size 1000 --run 1000
launch ./envelopeProducer --size 10   --run 1000
launch ./envelopeProducer --size 100  --run 1000
launch #./envelopeProducer --size 1000 --run 1000

launch # run using buffered port

launch ./envelopeProducer --size 100 --run 1000
launch ./envelopeProducer --size 100 --run 1000
launch ./envelopeProducer --size 100 --run 1000
