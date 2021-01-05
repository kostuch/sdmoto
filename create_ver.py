import datetime
Import("env")

curr_date = datetime.datetime.now()
year = str(curr_date.year)[2:]  # ignore first 2 chars of year; 2020 -> 20
# code it in YYMMDD
# day and month are zero-padded to length 2
date_str = f"{year}{curr_date.month:02}{curr_date.day:02}"
#print("-D FW_VERSION=",date_str)

# append integer value to global defines
env.Append(CPPDEFINES=[
    ("FW_VERSION", int(date_str))
])
