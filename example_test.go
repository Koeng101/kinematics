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

	// Math works slightly differently on arm and x86 machines when calculating
	// inverse kinematics. We check 5 decimals deep, since it appears numbers can
	// have slight variations between arm and x86 at 6 decimals.
	fmt.Printf("%5f, %5f, %5f, %5f, %5f, %5f\n", angles.J1, angles.J2, angles.J3, angles.J4, angles.J5, angles.J6)
	// Output: 1.846274, 0.341672, -2.313720, -1.776501, 2.221810, 1.231879
}
