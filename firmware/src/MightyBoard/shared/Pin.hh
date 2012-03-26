#ifndef PIN_HH
#define PIN_HH

#include "AvrPort.hh"
#include "Pin.hh"

/// \ingroup HardwareLibraries
class Pin {
private:
	// const AvrPort port;
	port_base_t port_base;
	bool is_null;
	uint8_t pin_index;
	uint8_t pin_mask;
	uint8_t pin_mask_inverted;

public:
	Pin();
	Pin(const AvrPort& port_in, uint8_t pin_index_in);
	bool isNull() const;
	void setDirection(bool out) const;
	bool getValue() const;
	void setValue(bool on) const;
	// currently not used:
	//const uint8_t getPinIndex() const { return pin_index; }
};

static const Pin NullPin(NullPort, 0);

#endif // PIN_HH
