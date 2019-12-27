import http.server
import rospy
from std_msgs.msg import String

class ServerHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):

        # rospy.init_node("worker_server_node")

        self.publisher = rospy.Publisher("/new_data", String, queue_size=128)
        rospy.loginfo("Finished initialization of ServerHandler")

        super().__init__( *args, **kwargs)


    def do_POST(self):
        content_len = int(self.headers['content-length'], 0)
        post_body = self.rfile.read(content_len)
        rospy.loginfo("{} {}".format(post_body, type(post_body)))
        to_publish = post_body.decode("utf-8")
        self.publisher.publish(String(to_publish))
        self.send_response(200)
        self.end_headers()

