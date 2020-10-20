from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import time
import datetime

def progress(count, total, starttime):
	bar_len = 50
	filled_len = int(round(bar_len * count / float(total)))

	percents = round(100.0 * count / float(total), 1)
	bar = '#' * filled_len + '.' * (bar_len - filled_len)
	elapsed = time.time() - starttime
	persecond = round(count / elapsed)
	if persecond:
		remainingseconds = datetime.datetime.utcfromtimestamp(
			(total - count) / persecond).strftime('%H:%M:%S')
	else:
		remainingseconds = 'n/a'

	sys.stdout.write('[%s] %s%s [elapsed: %s; remaining: %s; steps per second: %s]\r' % (
		bar,
		percents,
		'%',
		datetime.datetime.utcfromtimestamp(elapsed).strftime('%H:%M:%S'),
		remainingseconds,
		int(persecond)
	))
	sys.stdout.flush()

def end():
	sys.stdout.write("\n")
