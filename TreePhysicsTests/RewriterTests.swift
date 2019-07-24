import Foundation
import XCTest
@testable import TreePhysics
import SceneKit

class RewriterTests: XCTestCase {
    func testRewrite() {
        let rule1 = Rewriter.Rule(symbol: "b", replacement: "a")
        let rule2 = Rewriter.Rule(symbol: "a", replacement: "ab")
        XCTAssertEqual("b",
                       Rewriter.rewrite(premise: "b", rules: [rule1, rule2], generations: 0))
        XCTAssertEqual("a",
                       Rewriter.rewrite(premise: "b", rules: [rule1, rule2], generations: 1))
        XCTAssertEqual("ab",
                       Rewriter.rewrite(premise: "b", rules: [rule1, rule2], generations: 2))
        XCTAssertEqual("aba",
                       Rewriter.rewrite(premise: "b", rules: [rule1, rule2], generations: 3))
        XCTAssertEqual("abaab",
                       Rewriter.rewrite(premise: "b", rules: [rule1, rule2], generations: 4))
        XCTAssertEqual("abaababa",
                       Rewriter.rewrite(premise: "b", rules: [rule1, rule2], generations: 5))
    }
}
