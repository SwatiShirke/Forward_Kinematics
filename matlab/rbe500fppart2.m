%% rbe500 group assignment part 2
%% calc forward kinematics
  l0 = 36.076;
  l1 = 96.326-36.076;
  l2 = sqrt(128^2 + 24^2);
  l3 = 124;
  l4 = 133.4;
  q1 = 0;
  q2 = 0;
  q3=0;
  q4=0;
  A1 = calc_dh_trans(0,0,0,l0);
  A2 = calc_dh_trans(0,-pi/2,q1,l1);
  A3 = calc_dh_trans(l2,0,q2-pi/2,0);
  A4 = calc_dh_trans(l3,0,q3+pi/2,0);
  A5 = calc_dh_trans(l4,0,q4,0);
  T02 = A1*A2;
  T03 = T02*A3;
  T04 = T03*A4;
  T05 = T04*A5;

  %% calc jacobian
  o1 = A1(1:3,4);
  o2 = T02(1:3,4);
  o3 = T03(1:3,4);
  o4 = T04(1:3,4);
  o5 = T05(1:3,4);
  z = [0;0;1];
  z1 = A1(1:3,1:3)*z;
  z2 = T02(1:3,1:3)*z;
  z3 = T03(1:3,1:3)*z;
  z4 = T04(1:3,1:3)*z;
  z5 = T05(1:3,1:3)*z;

  J1 = [cross(z1,o5-o1);z1];
  J2 = [cross(z2,o5-o2);z2];
  J3 = [cross(z3,o5-o3);z3];
  J4 = [cross(z4,o5-o4);z4];

  J = [J1,J2,J3,J4];

  %% testing validity of jacobian
  test_joint_velo = [0.5;0;0;0];
  test_eef_velo = [0;400;0;0;0;0];
  inv_j = pinv(J);

  eef_velo = J*test_joint_velo
  joint_velo = inv_j*test_eef_velo

