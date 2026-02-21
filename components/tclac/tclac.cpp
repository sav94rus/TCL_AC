/**
 *
 **/
#include "esphome.h"
#include "esphome/core/defines.h"
#include "tclac.h"

namespace esphome{
namespace tclac{


ClimateTraits tclacClimate::traits() {
	auto traits = climate::ClimateTraits();

	traits.set_supports_action(false);
	traits.set_supports_current_temperature(true);
	traits.set_supports_two_point_target_temperature(false);

	// ESPHome 2026.x: FiniteSetMask заполняется через insert()
	climate::ClimateModeMask mode_mask;
	for (auto m : this->supported_modes_)
		mode_mask.insert(m);
	traits.set_supported_modes(mode_mask);

	climate::ClimatePresetMask preset_mask;
	for (auto p : this->supported_presets_)
		preset_mask.insert(p);
	traits.set_supported_presets(preset_mask);

	climate::ClimateFanModeMask fan_mask;
	for (auto f : this->supported_fan_modes_)
		fan_mask.insert(f);
	traits.set_supported_fan_modes(fan_mask);

	climate::ClimateSwingModeMask swing_mask;
	for (auto s : this->supported_swing_modes_)
		swing_mask.insert(s);
	traits.set_supported_swing_modes(swing_mask);

	traits.add_supported_mode(climate::CLIMATE_MODE_OFF);			// Выключенный режим кондиционера доступен всегда
	traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);			// Автоматический режим кондиционера тоже
	traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);		// Автоматический режим вентилятора доступен всегда
	traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);	// Выключенный режим качания заслонок доступен всегда
	traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);// На всякий случай без предустановок

	return traits;
}


void tclacClimate::setup() {

#ifdef CONF_RX_LED
	this->rx_led_pin_->setup();
	this->rx_led_pin_->digital_write(false);
#endif
#ifdef CONF_TX_LED
	this->tx_led_pin_->setup();
	this->tx_led_pin_->digital_write(false);
#endif
}

void tclacClimate::loop()  {
	// Если в буфере UART что-то есть, то читаем это что-то
	if (esphome::uart::UARTDevice::available() > 0) {
		dataShow(0, true);
		dataRX[0] = esphome::uart::UARTDevice::read();
		// Если принятый байт- не заголовок (0xBB), то просто покидаем цикл
		if (dataRX[0] != 0xBB) {
			ESP_LOGD("TCL", "Wrong byte");
			dataShow(0,0);
			return;
		}
		// А вот если совпал заголовок (0xBB), то начинаем чтение по цепочке еще 4 байт
		delay(5);
		dataRX[1] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[2] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[3] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[4] = esphome::uart::UARTDevice::read();

		//auto raw = getHex(dataRX, 5);
		//ESP_LOGD("TCL", "first 5 byte : %s ", raw.c_str());

		// Из первых 5 байт нам нужен пятый- он содержит длину сообщения
		esphome::uart::UARTDevice::read_array(dataRX+5, dataRX[4]+1);

		byte check = getChecksum(dataRX, sizeof(dataRX));

		//raw = getHex(dataRX, sizeof(dataRX));
		//ESP_LOGD("TCL", "RX full : %s ", raw.c_str());

		// Проверяем контрольную сумму
		if (check != dataRX[60]) {
			ESP_LOGD("TCL", "Invalid checksum %x", check);
			tclacClimate::dataShow(0,0);
			return;
		} else {
			//ESP_LOGD("TCL", "checksum OK %x", check);
		}
		tclacClimate::dataShow(0,0);
		// Прочитав все из буфера приступаем к разбору данных
		tclacClimate::readData();
	}
}

void tclacClimate::update() {
	tclacClimate::dataShow(1,1);
	this->esphome::uart::UARTDevice::write_array(poll, sizeof(poll));
	//auto raw = tclacClimate::getHex(poll, sizeof(poll));
	//ESP_LOGD("TCL", "chek status sended");
	tclacClimate::dataShow(1,0);
}

void tclacClimate::readData() {

	current_temperature = float((( (dataRX[17] << 8) | dataRX[18] ) / 374 - 32)/1.8);
	target_temperature = (dataRX[FAN_SPEED_POS] & SET_TEMP_MASK) + 16;

	//ESP_LOGD("TCL", "TEMP: %f ", current_temperature);

	if (dataRX[MODE_POS] & ( 1 << 4)) {
		uint8_t modeswitch = MODE_MASK & dataRX[MODE_POS];
		uint8_t fanspeedswitch = FAN_SPEED_MASK & dataRX[FAN_SPEED_POS];
		uint8_t swingmodeswitch = SWING_MODE_MASK & dataRX[SWING_POS];

		switch (modeswitch) {
			case MODE_AUTO:
				mode = climate::CLIMATE_MODE_AUTO;
				break;
			case MODE_COOL:
				mode = climate::CLIMATE_MODE_COOL;
				break;
			case MODE_DRY:
				mode = climate::CLIMATE_MODE_DRY;
				break;
			case MODE_FAN_ONLY:
				mode = climate::CLIMATE_MODE_FAN_ONLY;
				break;
			case MODE_HEAT:
				mode = climate::CLIMATE_MODE_HEAT;
				break;
			default:
				mode = climate::CLIMATE_MODE_AUTO;
		}

		if ( dataRX[FAN_QUIET_POS] & FAN_QUIET) {
			fan_mode = climate::CLIMATE_FAN_QUIET;
		} else if (dataRX[MODE_POS] & FAN_DIFFUSE){
			fan_mode = climate::CLIMATE_FAN_DIFFUSE;
		} else {
			switch (fanspeedswitch) {
				case FAN_AUTO:
					fan_mode = climate::CLIMATE_FAN_AUTO;
					break;
				case FAN_LOW:
					fan_mode = climate::CLIMATE_FAN_LOW;
					break;
				case FAN_MIDDLE:
					fan_mode = climate::CLIMATE_FAN_MIDDLE;
					break;
				case FAN_MEDIUM:
					fan_mode = climate::CLIMATE_FAN_MEDIUM;
					break;
				case FAN_HIGH:
					fan_mode = climate::CLIMATE_FAN_HIGH;
					break;
				case FAN_FOCUS:
					fan_mode = climate::CLIMATE_FAN_FOCUS;
					break;
				default:
					fan_mode = climate::CLIMATE_FAN_AUTO;
			}
		}

		switch (swingmodeswitch) {
			case SWING_OFF:
				swing_mode = climate::CLIMATE_SWING_OFF;
				break;
			case SWING_HORIZONTAL:
				swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
				break;
			case SWING_VERTICAL:
				swing_mode = climate::CLIMATE_SWING_VERTICAL;
				break;
			case SWING_BOTH:
				swing_mode = climate::CLIMATE_SWING_BOTH;
				break;
		}

		preset = ClimatePreset::CLIMATE_PRESET_NONE;
		if (dataRX[7] & (1 << 6)){
			preset = ClimatePreset::CLIMATE_PRESET_ECO;
		} else if (dataRX[9] & (1 << 2)){
			preset = ClimatePreset::CLIMATE_PRESET_COMFORT;
		} else if (dataRX[19] & (1 << 0)){
			preset = ClimatePreset::CLIMATE_PRESET_SLEEP;
		}

	} else {
		mode = climate::CLIMATE_MODE_OFF;
		swing_mode = climate::CLIMATE_SWING_OFF;
		preset = ClimatePreset::CLIMATE_PRESET_NONE;
	}
	this->publish_state();
	allow_take_control = true;
   }

// Climate control
void tclacClimate::control(const ClimateCall &call) {
	if (call.get_mode().has_value()){
		switch_climate_mode = call.get_mode().value();
		ESP_LOGD("TCL", "Get MODE from call");
	} else {
		switch_climate_mode = mode;
		ESP_LOGD("TCL", "Get MODE from AC");
	}

	if (call.get_preset().has_value()){
		switch_preset = call.get_preset().value();
	} else {
		switch_preset = preset.value();
	}

	if (call.get_fan_mode().has_value()){
		switch_fan_mode = call.get_fan_mode().value();
	} else {
		switch_fan_mode = fan_mode.value();
	}

	if (call.get_swing_mode().has_value()){
		switch_swing_mode = call.get_swing_mode().value();
	} else {
		switch_swing_mode = swing_mode;
	}

	if (call.get_target_temperature().has_value()) {
		target_temperature_set = 31-(int)call.get_target_temperature().value();
	} else {
		target_temperature_set = 31-(int)target_temperature;
	}

	is_call_control = true;
	takeControl();
	allow_take_control = true;
}


void tclacClimate::takeControl() {

	dataTX[7]  = 0b00000000;
	dataTX[8]  = 0b00000000;
	dataTX[9]  = 0b00000000;
	dataTX[10] = 0b00000000;
	dataTX[11] = 0b00000000;
	dataTX[19] = 0b00000000;
	dataTX[32] = 0b00000000;
	dataTX[33] = 0b00000000;

	if (is_call_control != true){
		ESP_LOGD("TCL", "Get MODE from AC for force config");
		switch_climate_mode = mode;
		switch_preset = preset.value();
		switch_fan_mode = fan_mode.value();
		switch_swing_mode = swing_mode;
		target_temperature_set = 31-(int)target_temperature;
	}

	if (beeper_status_){
		ESP_LOGD("TCL", "Beep mode ON");
		dataTX[7] += 0b00100000;
	} else {
		ESP_LOGD("TCL", "Beep mode OFF");
		dataTX[7] += 0b00000000;
	}

	if ((display_status_) && (switch_climate_mode != climate::CLIMATE_MODE_OFF)){
		ESP_LOGD("TCL", "Dispaly turn ON");
		dataTX[7] += 0b01000000;
	} else {
		ESP_LOGD("TCL", "Dispaly turn OFF");
		dataTX[7] += 0b00000000;
	}

	switch (switch_climate_mode) {
		case climate::CLIMATE_MODE_OFF:
			dataTX[7] += 0b00000000;
			dataTX[8] += 0b00000000;
			break;
		case climate::CLIMATE_MODE_AUTO:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00001000;
			break;
		case climate::CLIMATE_MODE_COOL:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000011;
			break;
		case climate::CLIMATE_MODE_DRY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000010;
			break;
		case climate::CLIMATE_MODE_FAN_ONLY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000111;
			break;
		case climate::CLIMATE_MODE_HEAT:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000001;
			break;
	}

	switch(switch_fan_mode) {
		case climate::CLIMATE_FAN_AUTO:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000000;
			break;
		case climate::CLIMATE_FAN_QUIET:
			dataTX[8]	+= 0b10000000;
			dataTX[10]	+= 0b00000000;
			break;
		case climate::CLIMATE_FAN_LOW:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000001;
			break;
		case climate::CLIMATE_FAN_MIDDLE:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000110;
			break;
		case climate::CLIMATE_FAN_MEDIUM:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000011;
			break;
		case climate::CLIMATE_FAN_HIGH:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000111;
			break;
		case climate::CLIMATE_FAN_FOCUS:
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000101;
			break;
		case climate::CLIMATE_FAN_DIFFUSE:
			dataTX[8]	+= 0b01000000;
			dataTX[10]	+= 0b00000000;
			break;
	}

	switch(switch_swing_mode) {
		case climate::CLIMATE_SWING_OFF:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_VERTICAL:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_HORIZONTAL:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00001000;
			break;
		case climate::CLIMATE_SWING_BOTH:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00001000;
			break;
	}

	switch(switch_preset) {
		case ClimatePreset::CLIMATE_PRESET_NONE:
			break;
		case ClimatePreset::CLIMATE_PRESET_ECO:
			dataTX[7]	+= 0b10000000;
			break;
		case ClimatePreset::CLIMATE_PRESET_SLEEP:
			dataTX[19]	+= 0b00000001;
			break;
		case ClimatePreset::CLIMATE_PRESET_COMFORT:
			dataTX[8]	+= 0b00010000;
			break;
	}

	switch(vertical_swing_direction_) {
		case VerticalSwingDirection::UP_DOWN:
			dataTX[32]	+= 0b00001000;
			ESP_LOGD("TCL", "Vertical swing: up-down");
			break;
		case VerticalSwingDirection::UPSIDE:
			dataTX[32]	+= 0b00010000;
			ESP_LOGD("TCL", "Vertical swing: upper");
			break;
		case VerticalSwingDirection::DOWNSIDE:
			dataTX[32]	+= 0b00011000;
			ESP_LOGD("TCL", "Vertical swing: downer");
			break;
	}
	switch(horizontal_swing_direction_) {
		case HorizontalSwingDirection::LEFT_RIGHT:
			dataTX[33]	+= 0b00001000;
			ESP_LOGD("TCL", "Horizontal swing: left-right");
			break;
		case HorizontalSwingDirection::LEFTSIDE:
			dataTX[33]	+= 0b00010000;
			ESP_LOGD("TCL", "Horizontal swing: lefter");
			break;
		case HorizontalSwingDirection::CENTER:
			dataTX[33]	+= 0b00011000;
			ESP_LOGD("TCL", "Horizontal swing: center");
			break;
		case HorizontalSwingDirection::RIGHTSIDE:
			dataTX[33]	+= 0b00100000;
			ESP_LOGD("TCL", "Horizontal swing: righter");
			break;
	}
	switch(vertical_direction_) {
		case AirflowVerticalDirection::LAST:
			dataTX[32]	+= 0b00000000;
			ESP_LOGD("TCL", "Vertical fix: last position");
			break;
		case AirflowVerticalDirection::MAX_UP:
			dataTX[32]	+= 0b00000001;
			ESP_LOGD("TCL", "Vertical fix: up");
			break;
		case AirflowVerticalDirection::UP:
			dataTX[32]	+= 0b00000010;
			ESP_LOGD("TCL", "Vertical fix: upper");
			break;
		case AirflowVerticalDirection::CENTER:
			dataTX[32]	+= 0b00000011;
			ESP_LOGD("TCL", "Vertical fix: center");
			break;
		case AirflowVerticalDirection::DOWN:
			dataTX[32]	+= 0b00000100;
			ESP_LOGD("TCL", "Vertical fix: downer");
			break;
		case AirflowVerticalDirection::MAX_DOWN:
			dataTX[32]	+= 0b00000101;
			ESP_LOGD("TCL", "Vertical fix: down");
			break;
	}
	switch(horizontal_direction_) {
		case AirflowHorizontalDirection::LAST:
			dataTX[33]	+= 0b00000000;
			ESP_LOGD("TCL", "Horizontal fix: last position");
			break;
		case AirflowHorizontalDirection::MAX_LEFT:
			dataTX[33]	+= 0b00000001;
			ESP_LOGD("TCL", "Horizontal fix: left");
			break;
		case AirflowHorizontalDirection::LEFT:
			dataTX[33]	+= 0b00000010;
			ESP_LOGD("TCL", "Horizontal fix: lefter");
			break;
		case AirflowHorizontalDirection::CENTER:
			dataTX[33]	+= 0b00000011;
			ESP_LOGD("TCL", "Horizontal fix: center");
			break;
		case AirflowHorizontalDirection::RIGHT:
			dataTX[33]	+= 0b00000100;
			ESP_LOGD("TCL", "Horizontal fix: righter");
			break;
		case AirflowHorizontalDirection::MAX_RIGHT:
			dataTX[33]	+= 0b00000101;
			ESP_LOGD("TCL", "Horizontal fix: right");
			break;
	}

	dataTX[9] = target_temperature_set;

	dataTX[0] = 0xBB;
	dataTX[1] = 0x00;
	dataTX[2] = 0x01;
	dataTX[3] = 0x03;
	dataTX[4] = 0x20;
	dataTX[5] = 0x03;
	dataTX[6] = 0x01;

	dataTX[12] = 0x00;
	dataTX[13] = 0x01;
	dataTX[14] = 0x00;
	dataTX[15] = 0x00;
	dataTX[16] = 0x00;
	dataTX[17] = 0x00;
	dataTX[18] = 0x00;
	dataTX[20] = 0x00;
	dataTX[21] = 0x00;
	dataTX[22] = 0x00;
	dataTX[23] = 0x00;
	dataTX[24] = 0x00;
	dataTX[25] = 0x00;
	dataTX[26] = 0x00;
	dataTX[27] = 0x00;
	dataTX[28] = 0x00;
	dataTX[30] = 0x00;
	dataTX[31] = 0x00;
	dataTX[34] = 0x00;
	dataTX[35] = 0x00;
	dataTX[36] = 0x00;

	dataTX[37] = 0xFF;
	dataTX[37] = tclacClimate::getChecksum(dataTX, sizeof(dataTX));

	tclacClimate::sendData(dataTX, sizeof(dataTX));
	allow_take_control = false;
	is_call_control = false;
}

void tclacClimate::sendData(byte * message, byte size) {
	tclacClimate::dataShow(1,1);
	this->esphome::uart::UARTDevice::write_array(message, size);
	ESP_LOGD("TCL", "Message to TCL sended...");
	tclacClimate::dataShow(1,0);
}

String tclacClimate::getHex(byte *message, byte size) {
	String raw;
	for (int i = 0; i < size; i++) {
		raw += "\n" + String(message[i]);
	}
	raw.toUpperCase();
	return raw;
}

byte tclacClimate::getChecksum(const byte * message, size_t size) {
	byte position = size - 1;
	byte crc = 0;
	for (int i = 0; i < position; i++)
		crc ^= message[i];
	return crc;
}

void tclacClimate::dataShow(bool flow, bool shine) {
	if (module_display_status_){
		if (flow == 0){
			if (shine == 1){
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(false);
#endif
			}
		}
		if (flow == 1) {
			if (shine == 1){
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(false);
#endif
			}
		}
	}
}

void tclacClimate::set_beeper_state(bool state) {
	this->beeper_status_ = state;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
void tclacClimate::set_display_state(bool state) {
	this->display_status_ = state;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
void tclacClimate::set_force_mode_state(bool state) {
	this->force_mode_status_ = state;
}
#ifdef CONF_RX_LED
void tclacClimate::set_rx_led_pin(GPIOPin *rx_led_pin) {
	this->rx_led_pin_ = rx_led_pin;
}
#endif
#ifdef CONF_TX_LED
void tclacClimate::set_tx_led_pin(GPIOPin *tx_led_pin) {
	this->tx_led_pin_ = tx_led_pin;
}
#endif
void tclacClimate::set_module_display_state(bool state) {
	this->module_display_status_ = state;
}
void tclacClimate::set_vertical_airflow(AirflowVerticalDirection direction) {
	this->vertical_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection direction) {
	this->horizontal_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection direction) {
	this->vertical_swing_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
void tclacClimate::set_supported_modes(const std::set<climate::ClimateMode> &modes) {
	this->supported_modes_ = modes;
}
void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection direction) {
	horizontal_swing_direction_ = direction;
	if (force_mode_status_){
		if (allow_take_control){
			tclacClimate::takeControl();
		}
	}
}
void tclacClimate::set_supported_fan_modes(const std::set<climate::ClimateFanMode> &modes){
	this->supported_fan_modes_ = modes;
}
void tclacClimate::set_supported_swing_modes(const std::set<climate::ClimateSwingMode> &modes) {
	this->supported_swing_modes_ = modes;
}
void tclacClimate::set_supported_presets(const std::set<climate::ClimatePreset> &presets) {
  this->supported_presets_ = presets;
}

}
}
