from numpy import cos, sin

def able_to_balance(theta1, theta2, theta3, h0, d3, a2, d4, m, a):
    s1 = sin(theta1)
    c1 = cos(theta1)
    s2 = sin(theta2)
    c2 = cos(theta2)
    s3 = sin(theta3)
    M0 = (0, 0)
    M1 = (d3*d3*c1/2, d3*d3*s1/2)
    M2 = (a2*(d3*c1+a2*s1*c2/2), a2*(d3*s1-a2*c1*c2/2))
    M3 = (d4*(d3*c1+a2*s1*c2+d4*s3/2), d4*(d3*s1-a2*c1*c2-d4*s3/2))
    Mm = (m*(d3*c1+a2*s1*c2+d4*s3), m*(d3*s1-a2*c1*c2-d4*s3))
    if M0[0]+M1[0]+M2[0]+M3[0]+Mm[0] > -a*(h0+d3+a2+d4+m) and M0[0]+M1[0]+M2[0]+M3[0]+Mm[0] < a*(h0+d3+a2+d4+m):
        if M0[1]+M1[1]+M2[1]+M3[1]+Mm[1] > -a*(h0+d3+a2+d4+m) and M0[1]+M1[1]+M2[1]+M3[1]+Mm[1] < a*(h0+d3+a2+d4+m):
            return True
    return False