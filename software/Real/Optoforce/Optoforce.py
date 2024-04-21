"""
Check : https://github.com/alknemeyer/optoforce/tree/main

Usage
    From a python script:
        ```py
        # 16 byte frame/single-channel 3 axis force sensor (OMD-45-FH-2000N)
        from optoforce import OptoForce16 as OptoForce
        from optoforce.status import no_errors

        with OptoForce(speed_hz=100, filter_hz=15, zero=False) as force_sensor:
            measurement = force_sensor.read(only_latest_data=False)
            assert measurement.valid_checksum
            assert no_errors(measurement.status)

            do_stuff_with_force_readings(measurement.Fx, measurement.Fy, measurement.Fz)
        ```
    A call to OptoForce.read() returns a measurement packet (a NamedTuple) containing force readings and other potentially useful data. For the specifics of each sensor model, see Reading16, Reading34 and Reading22 in optoforce/reading.py. For example OptoForce34 returns force readings as Fx1, Fy1, Fz1, Fx2, ... (not Fx, Fy, Fz) as there are multiple channels

    It's still a little verbose, so you may want to define shortcuts for your particular application. For example, if you don't care about anything except the vertical force:
        ```py
        with OptoForce() as force_sensor:
            read_fz = lambda: force_sensor.read(only_latest_data=False).Fz
            while True:
                print(read_fz())
        ```
    If you haven't read the force force sensor in a little while and want to get all the packets waiting in the buffer, use:
        ```py
        measurements = force_sensor.read_all_packets_in_buffer()
        ```
    Or from the command line, to log to a file:
        ```
        $ python -m optoforce --filename force-data.csv
        ```
    If you want to detect and handle sensor errors as reported in the status word, look at the doc string in optoforce/status.py. It was written for completeness after I finished my use for the OptoForce, so it hasn't been tested!
"""

# 16 byte frame/single-channel 3 axis force sensor (OMD-45-FH-2000N)
from software.Real.Optoforce import OptoForce16 as OptoForce
from software.Real.Optoforce.status import no_errors

with OptoForce(speed_hz=100, filter_hz=15, zero=False) as force_sensor:
    measurement = force_sensor.read(only_latest_data=False)
    assert measurement.valid_checksum
    assert no_errors(measurement.status)

    print(measurement.Fx, measurement.Fy, measurement.Fz)