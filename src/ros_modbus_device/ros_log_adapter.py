import logging
import rospy

class ROSLoggingAdapter(logging.Handler):
    mapping = {
        logging.DEBUG:rospy.logdebug,
        logging.INFO:rospy.loginfo,
        logging.WARNING:rospy.logwarn,
        logging.ERROR:rospy.logerr,
        logging.CRITICAL:rospy.logfatal
    }

    def emit(self, record):
        try:
            self.mapping[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))

    @staticmethod
    def attach(logger_name=__name__, log_level=logging.DEBUG):
        logger = logging.getLogger(logger_name)
        logger.addHandler(ROSLoggingAdapter())
        logger.setLevel(log_level)