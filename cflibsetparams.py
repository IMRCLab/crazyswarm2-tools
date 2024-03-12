from cflib.crazyflie import Crazyflie
import cflib.crtp
from cflib.utils.power_switch import PowerSwitch
import time

def on_fully_connected(link_uri):
    print('Connected to %s' % link_uri)
    cf.param.set_value('stabilizer.controller', 5)

    # 08.03.2024
    # cf.param.set_value('ctrlLee.Kpos_Px', 7.0)
    # cf.param.set_value('ctrlLee.Kpos_Py', 7.0)
    # cf.param.set_value('ctrlLee.Kpos_Pz', 20.0)

    # cf.param.set_value('ctrlLee.Kpos_Ix', 4.0)
    # cf.param.set_value('ctrlLee.Kpos_Iy', 4.0)
    # cf.param.set_value('ctrlLee.Kpos_Iz', 20.0)

    # cf.param.set_value('ctrlLee.Kpos_Dx', 6.0)
    # cf.param.set_value('ctrlLee.Kpos_Dy', 6.0)
    # cf.param.set_value('ctrlLee.Kpos_Dz', 20.0)

    # 11.03.2024 
    cf.param.set_value('ctrlLee.Kpos_Px', 6.0)
    cf.param.set_value('ctrlLee.Kpos_Py', 6.0)
    cf.param.set_value('ctrlLee.Kpos_Pz', 10.0)

    cf.param.set_value('ctrlLee.Kpos_Ix', 1.0)
    cf.param.set_value('ctrlLee.Kpos_Iy', 1.0)
    cf.param.set_value('ctrlLee.Kpos_Iz', 1.0)

    cf.param.set_value('ctrlLee.Kpos_Dx', 3.0)
    cf.param.set_value('ctrlLee.Kpos_Dy', 3.0)
    cf.param.set_value('ctrlLee.Kpos_Dz', 20.0)

    cf.param.set_value('ctrlLee.KR_x', 0.004)
    cf.param.set_value('ctrlLee.KR_y', 0.004)
    cf.param.set_value('ctrlLee.KR_z', 0.004)

    cf.param.set_value('ctrlLee.KI_x', 0.003)
    cf.param.set_value('ctrlLee.KI_y', 0.003)
    cf.param.set_value('ctrlLee.KI_z', 0.003)

    cf.param.set_value('ctrlLee.Kw_x', 0.001)
    cf.param.set_value('ctrlLee.Kw_y', 0.001)
    cf.param.set_value('ctrlLee.Kw_z', 0.001)

    cf.param.set_value('ctrlLee.mass', 0.032)

    print("done")
    # time.sleep(3)
    # cf.close_link()

cflib.crtp.init_drivers()

uri = "radio://0/80/2M/E7E7E7E70B"
PowerSwitch(uri).stm_power_cycle()
time.sleep(2)
cf = Crazyflie(rw_cache='./cache')
cf.fully_connected.add_callback(on_fully_connected)
cf.open_link(uri)

