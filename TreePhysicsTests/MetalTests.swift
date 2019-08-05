import Foundation
import XCTest
@testable import TreePhysics
import MetalKit


class MetalTests: XCTestCase {
    var foo: Foo!

    override func setUp() {
        super.setUp()
        self.foo = Foo()
    }

    let matrix = float3x3(columns: (
        float3(2,1,0),
        float3(1,2,1),
        float3(0,1,2)))


    func testSetup() {
        let expectation = XCTestExpectation(description: "wait")
        foo.run(matrix) { buffer in
            let foo = UnsafeMutableRawPointer(buffer.contents()).bindMemory(to: float3.self, capacity: 1024)
            XCTAssertEqual(
                float3(2 + sqrt(2), 2, 2 - sqrt(2.0)),
                foo[0], accuracy: 0.0001)
            expectation.fulfill()
        }
    }
    //
    //    func testFoo() {
    //
    //
    //        guard let (eigenvalues, eigenvectors) = matrix.eigen_ql else {
    //            XCTFail()
    //            return
    //        }
    //
    //        XCTAssertEqual(
    //            double3(2 - sqrt(2), 2, 2 + sqrt(2.0)),
    //            eigenvalues, accuracy: 0.0001)
    //
    //        XCTAssertEqual(
    //            double3x3(columns: (
    //                -double3(1.0 / 2, -sqrt(2.0) / 2, 1.0 / 2),
    //                double3(1 / sqrt(2.0), 0, -1/sqrt(2.0)),
    //                double3(1.0/2, sqrt(2.0) / 2, 1.0/2)
    //            )),
    //            eigenvectors, accuracy: 0.001)
    //    }

}
