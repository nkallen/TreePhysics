import Foundation
import Cocoa

public typealias Settings = [(String, Float, Float, Float, Any?, Selector)]

public class SettingsView: NSStackView {
    public init(settings: Settings, frame: CGRect) {
        super.init(frame: frame)

        self.orientation = .vertical
//        self.translatesAutoresizingMaskIntoConstraints = false
        self.distribution = .fill

        for (stringValue, floatValue, min, max, target, selector) in settings {
            let stackView = NSStackView()
            stackView.translatesAutoresizingMaskIntoConstraints = false
            stackView.distribution = .fill

            let label = NSTextField()
            label.stringValue = stringValue
            label.isEditable = false
            label.isBezeled = false

            let slider = NSSlider(value: Double(floatValue), minValue: Double(min), maxValue: Double(max), target: target, action: selector)

            stackView.addArrangedSubview(label)
            stackView.addArrangedSubview(slider)

            addArrangedSubview(stackView)
        }
    }

    required init?(coder decoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
