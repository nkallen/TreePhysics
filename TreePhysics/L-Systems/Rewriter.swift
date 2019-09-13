import Foundation

// L-system rewriter

public class Rewriter {
    public struct Rule {
        public let symbol: Character
        public let replacement: String

        public init(symbol: Character, replacement: String) {
            self.symbol = symbol
            self.replacement = replacement
        }

        func matches(_ character: Character) -> Bool {
            return symbol == character
        }
    }

    public static func rewrite(premise: String, rules: [Rule], generations: Int) -> String {
        var premise = premise
        for _ in 0..<generations {
            let unchanged = premise
            var i = premise.startIndex
            while i < premise.endIndex {
                let char = premise[i]
                if let rule = matches(rules: rules, character: char) {
                    let start = premise[premise.startIndex..<i]
                    let middle = rule.replacement
                    let end = premise[premise.index(after: i)..<premise.endIndex]
                    premise = start + middle + end
                    i = premise.index(i, offsetBy: rule.replacement.count)
                } else {
                    i = premise.index(after: i)
                }
            }
            if premise == unchanged { break }
        }
        return premise
    }

    private static func matches(rules: [Rule], character: Character) -> Rule? {
        for rule in rules {
            if rule.matches(character) {
                return rule
            }
        }
        return nil
    }
}

