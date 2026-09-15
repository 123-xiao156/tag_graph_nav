from math import pi, fmod, fabs


def diffAng(target_ang, source_ang):
    a = target_ang - source_ang
    #a = fmod(a + pi, 2*pi) - pi
    #return a+2*pi if a <= -pi else a
    a = fmod(a,2*pi)
    if a <= -pi:
      return a+2*pi
    elif a > pi:
      return a-2*pi
    return a


# from_ang: 机器人当前朝向
# route_ang: 路径朝向
# to_ang: 目标朝向
# return: 路径朝向与当前朝向的差值，取较小值
def calcRoute(from_ang, route_ang, to_ang):
    reverse_route_ang = route_ang + pi
    route_dist = fabs(diffAng(route_ang, from_ang)) + \
        fabs(diffAng(to_ang, route_ang))
    reverse_dist = fabs(diffAng(reverse_route_ang, from_ang)) + \
        fabs(diffAng(to_ang, reverse_route_ang))
    return diffAng(route_ang, from_ang) if route_dist <= reverse_dist else diffAng(reverse_route_ang, from_ang)
