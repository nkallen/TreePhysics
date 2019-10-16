import Foundation
import Cocoa

public typealias SettingChangedCallback = (Float) -> ()

public class SettingsView: NSStackView {
    var callbacks: [NSSlider:SettingChangedCallback] = [:]

    public override init(frame: CGRect) {
        super.init(frame: frame)

        self.orientation = .vertical
//        self.translatesAutoresizingMaskIntoConstraints = false
        self.distribution = .fill
    }

    public func add(_ name: String, _ value: Float, _ min: Float, _ max: Float, cb: @escaping SettingChangedCallback) {
        let stackView = NSStackView()
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.distribution = .fill

        let label = NSTextField()
        label.stringValue = name
        label.isEditable = false
        label.isBezeled = false

        let slider = NSSlider(value: Double(value), minValue: Double(min), maxValue: Double(max), target: self, action: #selector(settingDidChange))

        stackView.addArrangedSubview(label)
        stackView.addArrangedSubview(slider)

        addArrangedSubview(stackView)

        callbacks[slider] = cb
    }

    @objc func settingDidChange(sender: NSSlider) {
        let callback = callbacks[sender]!
        print(sender.floatValue)
        callback(sender.floatValue)
    }

    required init?(coder decoder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
}
