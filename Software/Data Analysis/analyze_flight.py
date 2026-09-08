import csv
rows = list(csv.DictReader(open('RapidFishFW/flight_data_1788545648_core.csv')))
pressures = [float(r['Pressure_Pa']) for r in rows]
alts = [float(r['Altitude_m']) for r in rows]
print('Total rows:', len(rows))
print('Pressure range:', min(pressures), '-', max(pressures), 'Pa')
print('Altitude range:', min(alts), '-', max(alts), 'm')
print('Start alt:', alts[0], 'End alt:', alts[-1])
# Check first negative
for i,r in enumerate(rows):
    if float(r['Altitude_m']) < 0:
        print('First negative alt at row', i, 'ts=', r['Timestamp_ms'], 'press=', r['Pressure_Pa'], 'state=', r['State'])
        break
# Check state transitions
prev = rows[0]['State']
for i,r in enumerate(rows):
    if r['State'] != prev:
        print('State change at row', i, 'ts=', r['Timestamp_ms'], 'alt=', r['Altitude_m'], ':', prev, '->', r['State'])
        prev = r['State']