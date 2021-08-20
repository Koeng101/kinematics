package kinematics_test

import (
	"fmt"
	"github.com/koeng101/armos/utils/kinematics"
)

func ExampleForwardKinematics() {
	angles := kinematics.StepperTheta{J1: 10, J2: 1, J3: 1, J4: 0, J5: 0, J6: 0}
	coordinates := kinematics.ForwardKinematics(angles, kinematics.AR3DhParameters)

	fmt.Println(coordinates)
	// Output: {-101.74590611879692 -65.96805988175777 -322.27756822304093 0.06040824945687102 -0.20421099379003957 0.2771553334491873 0.9369277637862541}
}

func ExampleInverseKinematics() {
	coordinates := kinematics.XyzWxyz{X: -100, Y: 250, Z: 250, Qx: 0.4007833787652043, Qy: -0.021233218878182854, Qz: 0.9086418268616911, Qw: 0.41903052745255764}
	angles, _ := kinematics.InverseKinematics(coordinates, kinematics.AR3DhParameters)

	fmt.Println(angles)
	// Output: {1.8462740950010432 0.3416721655970939 -2.313720459511564 -1.7765008677785283 2.2218097507147707 1.2318789996199948}
}
