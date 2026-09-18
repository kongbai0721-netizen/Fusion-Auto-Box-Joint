from .fusion_auto_box_joint import AutoBoxJointAddIn
from .fusion_util import log, handleException


log(f'Loading Fusion Add-In {__file__!r} ...')


autoBoxJointAddIn: AutoBoxJointAddIn = None


def run(context):
	try:
		log(f'Starting Fusion Add-In {__file__!r} ...')
		global autoBoxJointAddIn
		autoBoxJointAddIn = AutoBoxJointAddIn()
	except:
		handleException()


def stop(context):
	try:
		log(f'Stopping Fusion Add-In {__file__!r} ...')
		global autoBoxJointAddIn
		if autoBoxJointAddIn:
			autoBoxJointAddIn.stop()
			autoBoxJointAddIn = None
	except:
		handleException()


log(f'Finished loading Fusion Add-In {__file__!r}')
