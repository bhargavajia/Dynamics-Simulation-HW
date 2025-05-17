function Rdot = Rdot_eq(t, R, w)

    S_w = [  0   -w(3)  w(2);
            w(3)   0   -w(1);
           -w(2)  w(1)   0  ];

    Rdot = S_w * R;
end
