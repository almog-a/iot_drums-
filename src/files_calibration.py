def get_calibration_from_file(filename):
    lower = []
    upper = []
    with open(filename) as f:
        lines = f.read().split()

    for i in range(3):
        lower.append(int(lines[i].strip("''")))
        upper.append(int(lines[i + 3].strip("''")))
    return lower,upper



def set_calibration_to_file(filename,lower,upper):

    #lower=[1,2,3]
    #upper=[100,101,102]
    textFile=open(filename,"w")
    #total=lower+upper
    for element in lower:
        textFile.write(str(element))
        textFile.write(" ")
    for element in upper:
        textFile.write(str(element))
        textFile.write(" ")

    textFile.close()