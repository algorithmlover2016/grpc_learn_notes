import csv
with open('eggs.csv', 'w', newline='') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=' ',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    spamwriter.writerow(['Spam'] * 5 + ['Baked Beans'])
    spamwriter.writerow(['Spam', 'Lovely Spam', 'Wonderful Spam'])

import csv
with open('eggs.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        print(', '.join(row))

import csv
with open('names.csv', 'w', newline='') as csvfile:
    fieldnames = ['first_name', 'last_name']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    writer.writerow({'first_name': 'Baked', 'last_name': 'Beans'})
    writer.writerow({'first_name': 'Lovely', 'last_name': 'Spam'})
    writer.writerow({'first_name': 'Wonderful', 'last_name': 'Spam'})

import csv
with open('names.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        print(row)
        # print(row['first_name'], row['last_name'])

'''
csv模块已经内置了3个属性集：

csv.excel：采用逗号分隔的Excel格式csv
    delimiter = ','
    doublequote = True
    lineterminator = '\r\n'
    quotechar = '"'
    quoting = 0
    skipinitialspace = False
    escapechar = None
csv.excel_tab：采用tab分隔的Excel格式csv
    delimiter = '\t' #除分隔符外，其他继承自csv.excel类
    csv.unix_dialect：采用逗号分隔的Unix格式csv
    delimiter = ','
    doublequote = True
    lineterminator = '\n'
    quotechar = '"'
    quoting = 1
    skipinitialspace = False
    escapechar = None
'''

l = []
with open('test1.csv','rt', encoding="utf-8") as f: 
   cr = csv.reader(f, skipinitialspace = True)
   for row in cr:
       print(row)
       l.append(row) #将test.csv内容读入列表l，每行为其一个元素，元素也为list

with open('1.csv','wt', encoding="utf-8") as f2:
   cw = csv.writer(f2)
   #采用writerow()方法
   for item in l:
      cw.writerow(item) #将列表的每个元素写到csv文件的一行
   #或采用writerows()方法
   cw.writerows(l) #将嵌套列表内容写入csv文件，每个外层元素为一行，每个内层元素为一个数据

l = []
with open('test1.csv','rt', encoding="utf-8") as f: 
   cr = csv.DictReader(f, skipinitialspace = True)
   for row in cr:
       print(row)
       l.append(row) #将test2.csv内容读入列表l，每行为其一个元素，元素为dict
                     #key为标题（未指定时为读取的第一行），value为对应列的数据
print(l)
with open('2.csv','wt', encoding="utf-8") as f2:
   cw = csv.DictWriter(f2, fieldnames=['标题%d' % i for i in range(1,7)], lineterminator = '\n')
   cw.writeheader() #将fieldnames写入标题行
   #采用writerow()方法
   for rowdict in l:
      cw.writerow(rowdict) #将列表的每个元素（dict）按照对应的键值对写到csv文件的一行
   #或采用writerows()方法
   cw.writerows(l) #将dict组成的list整体写入csv文件，每个dict为一行，每个value为一个数据