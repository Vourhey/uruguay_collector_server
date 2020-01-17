import http.server
import rospy
import json
from std_msgs.msg import String

import sqlalchemy as db


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

class DBHelper:
    def __init__(self, db_url):
        engine = db.create_engine(rospy.get_param("~db_url")[:-1])
        self.conn = engine.connect()

        self.measurement_table, self.last_row_table = self.__init_db(engine)

    def __init_db(self, engine) -> tuple:
        metadata = db.MetaData()
        table_name = "measurement"
        if not engine.dialect.has_table(engine, table_name):
            rospy.loginfo("Initialazing DB with a table name '{}'".format(table_name))

            measurement_table = db.Table(table_name, metadata,
                             db.Column("id", db.Integer, primary_key=True),
                             db.Column("measure", db.TEXT),
                             db.Column("time_stamp", db.TIMESTAMP(True), server_default=db.sql.func.now()))
            last_row_table = db.Table("last_row", metadata,
                             db.Column("id", db.Integer, primary_key=True),
                             db.Column("row", db.Integer))
            metadata.create_all(engine)

            ins = db.insert(last_row_table).values(row=0)
            res = self.conn.execute(ins)

        measurement_table = db.Table(table_name, metadata, autoload=True, autoload_with=engine)
        last_row_table = db.Table("last_row", metadata, autoload=True, autoload_with=engine)
        return (measurement_table, last_row_table)

    def insert_row(self, data: str):
        ins = db.insert(self.measurement_table).values(measure=data)
        res = self.conn.execute(ins)

    def collect_data(self) -> str:
        rospy.loginfo("Collecting data...")
        sel = self.last_row_table.select("row").order_by("id", db.desc("id")).limit(1)
        res = self.conn.execute(sel).fetchone()

        rospy.loginfo(res)

        last = int(res["row"])

        sel = self.measurement_table.select().where(self.measurement_table.id >= last)
        res = self.conn.execute(sel).fetchall()

        data = {}
        for i in res:
            data[i[2]] = i[1]   # "timestamp": "row"
            last = i[0]

        ins = db.insert(self.last_row_table).values(row=last)
        res = self.conn.execute(ins)

        return json.dumps(data)

