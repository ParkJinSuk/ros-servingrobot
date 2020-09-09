import pymysql

my_db = pymysql.connect(
    user='sexymandoo',
    passwd='sexymandoo',
    host='101.101.218.239',
    db='project',
    charset='utf8'
)

cursor = my_db.cursor(pymysql.cursors.DictCursor)

sql_all = "SELECT * FROM `orderDB`;"
sql_callArduino = "SELECT _table FROM project.orderDB WHERE call_arduino = '1' AND serving = '0';"
sql_completecall = "UPDATE project.orderDB SET serving = '1' WHERE call_arduino = '1' AND serving = '0';"
cursor.execute(sql_callArduino)
result = cursor.fetchall()
order_table_num = result[0]['_table']

print(order_table_num)

cursor.execute(sql_completecall)
print("complete")
my_db.commit()