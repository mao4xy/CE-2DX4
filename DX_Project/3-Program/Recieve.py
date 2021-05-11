import serial
import xlsxwriter

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.


#Python 3.88

# create excel workbook, add worksheet
workbook = xlsxwriter.Workbook("dataset.xlsx")
worksheet = workbook.add_worksheet()
# format excel sheet
worksheet.write(0, 0, "Motor Position")
# headers
for col in range(1, 11):
    worksheet.write(0, col, "x(mm)=")
    worksheet.write(1, col, 100*(col-1))
# rotation step
for row in range(2, 66):
    worksheet.write(row, 0, row-1)

s = serial.Serial("COM3", 115200)
print("Opening: " + s.name)    

for col in range(1, 11):
    for row in range(2, 66):
        dist = ""
        while True: # will continue until a full measurement has been read (i.e. will hang in between push button presses)
            x = s.read()        # read one byte
            c = x.decode()      # convert byte type to str

            if c == 'E':    #E is the end bit sent at the end of every measurement
                break

            dist += c   #read each digit of the measurement one at a time
        print(dist);
        worksheet.write(row, col, dist)
        
print("Closing: " + s.name)
workbook.close()
s.close()
