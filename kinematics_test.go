package kinematics

import (
	"fmt"
	"testing"

	"gonum.org/v1/gonum/mat"
)

func TestApproxEquals(t *testing.T) {
	p0 := Position{1, 2, 3}
	p1 := Position{1 + 1.1e-6, 2 - 3e-7, 3 + 2.45e-6}
	tol := 1e-5
	if !p0.ApproxEqual(p1, tol) {
		t.Errorf("ApproxEqual failed at tol %f with p0: %+v, p1: %+v", tol, p0, p1)
	}
	tol = 1e-6
	if p0.ApproxEqual(p1, tol) {
		t.Errorf("ApproxEqual failed at tol %f with p0: %+v, p1: %+v", tol, p0, p1)
	}

	q0 := Quaternion{1, 2, 3, 4}
	q1 := Quaternion{-1, -2, -3, -4}
	if !q0.ApproxEqual(q1, tol) {
		t.Errorf("ApproxEqual failed for negative quaternions q0: %+v, q1: %+v", q0, q1)
	}

	tol = 1e-5
	q1 = Quaternion{-1 + 2.345e-6, -2 - 4.3e-7, -3 + 2e-6, -4}
	if !q0.ApproxEqual(q1, tol) {
		t.Errorf("ApproxEqual failed for close negative quaternions q0: %+v, q1: %+v", q0, q1)
	}
	q1 = Quaternion{1 + 2.345e-6, 2 - 4.3e-7, 3 + 2e-6, 4}
	if !q0.ApproxEqual(q1, tol) {
		t.Errorf("ApproxEqual failed for close quaternions q0: %+v, q1: %+v", q0, q1)
	}

	tol = 1e-7
	q1 = Quaternion{1 + 2.345e-6, 2 - 4.3e-7, 3 + 2e-6, 4}
	if q0.ApproxEqual(q1, tol) {
		t.Errorf("ApproxEqual failed for close quaternions q0: %+v, q1: %+v", q0, q1)
	}
}

func TestForwardKinematics(t *testing.T) {
	testThetas := []float64{10, 1, 1, 0, 0, 0}
	f := ForwardKinematics(testThetas, AR3DhParameters)
	switch {
	case f.Pos.X != -101.74590611879692:
		t.Errorf("Forward kinematics failed on f.Pos.X = %f", f.Pos.X)
	case f.Pos.Y != -65.96805988175777:
		t.Errorf("Forward kinematics failed on f.Pos.Y = %f", f.Pos.Y)
	case f.Pos.Z != -322.27756822304093:
		t.Errorf("Forward kinematics failed on f.Pos.Z = %f", f.Pos.Z)
	case f.Rot.X != 0.06040824945687102:
		t.Errorf("Forward kinematics failed on f.Rot.X = %f", f.Rot.X)
	case f.Rot.Y != -0.20421099379003957:
		t.Errorf("Forward kinematics failed on f.Rot.Y = %f", f.Rot.Y)
	case f.Rot.Z != 0.2771553334491873:
		t.Errorf("Forward kinematics failed on f.Rot.Z = %f", f.Rot.Z)
	case f.Rot.W != 0.9369277637862541:
		t.Errorf("Forward kinematics failed on f.Rot.W = %f", f.Rot.W)
	}
}

func TestInverseKinematics(t *testing.T) {
	thetasInit := []float64{0, 0, 0, 0, 0, 0}
	desiredEndEffector := Pose{
		Position{
			-91.72345062922584,
			386.93155027870745,
			382.30917872225154},
		Quaternion{
			W: 0.41903052745255764,
			X: 0.4007833787652043,
			Y: -0.021233218878182854,
			Z: 0.9086418268616911}}
	_, err := InverseKinematics(desiredEndEffector, AR3DhParameters, thetasInit)
	if err != nil {
		t.Errorf("Inverse Kinematics failed with error: %s", err)
	}

	// This case should fail because the X required is too large
	desiredEndEffector = Pose{
		Position{-91000000.72345062922584,
			386.93155027870745,
			382.30917872225154},
		Quaternion{
			W: 0.41903052745255764,
			X: 0.4007833787652043,
			Y: -0.021233218878182854,
			Z: 0.9086418268616911}}
	_, err = InverseKinematics(desiredEndEffector, AR3DhParameters, thetasInit)
	if err == nil {
		t.Errorf("Inverse Kinematics should have failed with large X")
	}
}

func TestInverseKinematicsSevDOF(t *testing.T) {
	thetasInit := []float64{0, 0, 0, 0, 0, 0, 0}

	thetasTarg := []float64{0, 0, 0, 0, 0, 0, 0}
	for i := range thetasTarg {
		thetasTarg[i] = RandTheta() * 0.1
	}
	pTarg := ForwardKinematics(thetasTarg, SevDOFDhParameters)

	thetasSolve, err := InverseKinematics(pTarg, SevDOFDhParameters, thetasInit)
	if err != nil {
		t.Errorf("Inverse Kinematics failed with error: %s", err)
	}
	pSolve := ForwardKinematics(thetasSolve, SevDOFDhParameters)

	if !pSolve.ApproxEqual(pTarg, 1e-3) {
		t.Errorf("Inverse Kinematics didn't arrive at the correct"+
			" solution: %+v", pSolve)
	}
}

func TestMatrixToQuaterion(t *testing.T) {
	var q Quaternion

	// Test tr > 0
	accumulatortMat1 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	q = MatrixToQuaterion(accumulatortMat1)
	switch {
	case q.W != 1:
		t.Errorf("Failed mat1 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat1 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat1 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat1 with q.Z = %f", q.Z)
	}

	// Test (accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1)) &&
	// (accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2))
	accumulatortMat2 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 0,
	})
	q = MatrixToQuaterion(accumulatortMat2)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat2 with qw = %f", q.W)
	case q.X != 1:
		t.Errorf("Failed mat2 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat2 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat2 with q.Z = %f", q.Z)
	}

	// Test accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2)
	accumulatortMat3 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, -2, 0,
		0, 0, 0, 1,
	})
	q = MatrixToQuaterion(accumulatortMat3)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat3 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat3 with q.X = %f", q.X)
	case q.Y != 1:
		t.Errorf("Failed mat3 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat3 with q.Z = %f", q.Z)
	}

	// Test default
	accumulatortMat4 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, -2, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1,
	})
	q = MatrixToQuaterion(accumulatortMat4)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat4 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat4 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat4 with q.Y = %f", q.Y)
	case q.Z != 1:
		t.Errorf("Failed mat4 with q.Z = %f", q.Z)
	}
}

func BenchmarkInverseKinematics(b *testing.B) {
	thetasInit := []float64{0, 0, 0, 0, 0, 0}

	for i := 0; i < b.N; i++ {
		randomSeed := []float64{RandTheta(), RandTheta(), RandTheta(),
			RandTheta(), RandTheta(), RandTheta()}
		desiredEndEffector := ForwardKinematics(randomSeed, AR3DhParameters)
		_, err := InverseKinematics(desiredEndEffector, AR3DhParameters, thetasInit)
		if err != nil {
			b.Errorf("Failed inverse kinematics benchmark with: %s\nSeed:"+
				" %f-%f-%f-%f-%f-%f", err,
				randomSeed[0], randomSeed[1], randomSeed[2],
				randomSeed[3], randomSeed[4], randomSeed[5])
		}
	}
}

func ExampleForwardKinematics() {
	angles := []float64{10, 1, 1, 0, 0, 0}
	coordinates := ForwardKinematics(angles, AR3DhParameters)

	fmt.Println(coordinates)
	// Output: {{-101.74590611879692 -65.96805988175777 -322.27756822304093} {0.9369277637862541 0.06040824945687102 -0.20421099379003957 0.2771553334491873}}
}

func ExampleInverseKinematics() {
	coordinates := Pose{Position{X: -100, Y: 250, Z: 250},
		Quaternion{
			X: 0.4007833787652043,
			Y: -0.021233218878182854,
			Z: 0.9086418268616911,
			W: 0.41903052745255764}}
	initThetas := []float64{0, 0, 0, 0, 0, 0}
	angles, _ := InverseKinematics(coordinates, AR3DhParameters, initThetas)

	fmt.Println(angles)
	// Output: [1.8462740950010432 0.3416721655970939 -2.313720459511564 -1.7765008677785283 2.2218097507147707 1.2318789996199948]
}
