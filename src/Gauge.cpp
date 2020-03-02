#include <stdlib.h>
#include "Gauge.h"
//#include "ILI9341.h"

//void Gauge::update(int16_t) {}
//void Gauge::show(void) {}

// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
uint16_t Gauge::rainbow(uint8_t value)
{
	// Value is expected to be in range 0-127
	// The value is converted to a spectrum colour from 0 = blue through to 127 = red
	byte red = 0; // Red is the top 5 bits of a 16 bit colour value
	byte green = 0;// Green is the middle 6 bits
	byte blue = 0; // Blue is the bottom 5 bits
	byte quadrant = value / 32;

	if (quadrant == 0)
	{
		blue = 31;
		green = 2 * (value % 32);
		red = 0;
	}

	if (quadrant == 1)
	{
		blue = 31 - (value % 32);
		green = 63;
		red = 0;
	}

	if (quadrant == 2)
	{
		blue = 0;
		green = 63;
		red = value % 32;
	}

	if (quadrant == 3)
	{
		blue = 0;
		green = 63 - 2 * (value % 32);
		red = 31;
	}

	return (red << 11) + (green << 5) + blue;
}

HGauge::HGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c,
               uint16_t fg_c, bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val)
{
	_gfx_dev = gfx_dev;
	_g_def._x = x;
	_g_def._y = y;
	_g_def._w = w;
	_g_def._h = h;
	_g_def._back_c = back_c;
	_g_def._frame_c = frame_c;
	_g_def._fg_c = fg_c;
	_g_def._val_txt = val_txt;
	_g_def._label = (char *) label;
	_g_def._txt_size = txt_size;
	_g_def._style = gstyle;
	_g_def._min_val = min_val;
	_g_def._max_val = max_val;
	_g_def._value = min_val;
}

void HGauge::show(void)
{
	// Ramka (pusta)
	_gfx_dev->drawRect(_g_def._x, _g_def._y, _g_def._w, _g_def._h, _g_def._frame_c);
	_gfx_dev->fillRect(_g_def._x + 1, _g_def._y + 1, _g_def._w - 2, _g_def._h - 2, _g_def._back_c);
	// Zawartosc wskaznika (poczatkowa)
	update(_g_def._value);
}

void HGauge::update(int16_t value)
{
	// Ograniczenie wartosci i mapowanie na szerokosc wskaznika
	uint8_t gval = map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 0, _g_def._w - 4);
	bool newval_lower;

	// Jezeli nowa wwartosc wskaznika mniejsza od wyswietlanej, to zamaz wskazanie (tylko powyzej poprzedniej wartosci)
	if (value < _g_def._value)
	{
		newval_lower = true;
		_gfx_dev->fillRect(gval + _g_def._x + 1, _g_def._y + 1, _g_def._w - gval - 2, _g_def._h - 2, _g_def._back_c);
	}
	else newval_lower = false;

	_g_def._value = value;
	uint8_t segs = _g_def._w / SEGMENT_S;										// Ilosc "segmentow" we wskazniku
	uint8_t snum = map(value, _g_def._min_val, _g_def._max_val, 0, segs);		// Ilosc segmentow odpowiadajaca wartosci
	char buf[8];

	if (_g_def._val_txt) itoa(value, buf, 10);									// Jezeli flaga val_txt jest ustawiona, to opisem jest wartosc

	uint16_t colour;

	switch (_g_def._style)														// W zaleznosci od stylu wskaznika rysuj zawartosc
	{
		case G_PLAIN:
			// Wskaznik (0, 0, 100, 6)
			//  0         1         2         3         4         5         6         7         8         9
			//  0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789
			// 0----------------------------------------------------------------------------------------------------
			// 1|                                                                                                  |
			// 2||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
			// 3||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
			// 4||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
			// 5|                                                                                                  |
			// 6----------------------------------------------------------------------------------------------------
			colour = _g_def._fg_c;

			if (!newval_lower) fill_gauge(colour, gval);						// Jezeli nowa wartosc wieksza od poprzedniej rysuj zawartosc

			break;

		case G_SEGMENTS:

			// Wskaznik (0, 0, 100, 6)
			//  0         1         2         3         4         5         6         7         8         9
			//  0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789
			// 0----------------------------------------------------------------------------------------------------
			// 1|                                                                                                  |
			// 2|||||||||||    ||||||||||    |||||||||||    |||||||||||    |||||||||||    |||||||||||    |||||||||||
			// 3|||||||||||    ||||||||||    |||||||||||    |||||||||||    |||||||||||    |||||||||||    |||||||||||
			// 4|||||||||||    ||||||||||    |||||||||||    |||||||||||    |||||||||||    |||||||||||    |||||||||||
			// 5|                                                                                                  |
			// 6----------------------------------------------------------------------------------------------------
			if (!newval_lower)													// Jezeli nowa wartosc wieksza od poprzedniej rysuj zawartosc
			{
				for (uint8_t seg = 0; seg < snum; seg++) _gfx_dev->fillRect(_g_def._x + 2 + (seg * SEGMENT_S), _g_def._y + 2, SEGMENT_W, _g_def._h - 4, _g_def._fg_c);
			}

			break;

		case G_B2R:
			colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 0, 127)); // Spectrum B -> R
			fill_gauge(colour, gval);											// Rysuj zawartosc
			break;

		case G_G2R:
			colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 63, 127)); // Spectrum G -> R
			fill_gauge(colour, gval);											// Rysuj zawartosc
			break;

		case G_R2G:
			colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 127, 63)); // Spectrum R -> G
			fill_gauge(colour, gval);											// Rysuj zawartosc
			break;

		case G_M_B2R:
		case G_M_R2G:
		case G_M_G2R:
			fill_gauge(_g_def._style, gval);
			break;

		default:
			break;
	}

	_gfx_dev->setTextFont(1);
	_gfx_dev->setTextColor(_g_def._frame_c, _g_def._back_c);
	_gfx_dev->setTextSize(_g_def._txt_size);
	// Kursor dla samej wartosci
	_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(buf) + 2) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));

	if (strlen(_g_def._label))
	{
		if (_g_def._val_txt)
		{
			// Kursor dla etykieta z wartoscia
			_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(_g_def._label) + strlen(buf) + 3) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));
		}
		else
		{
			// Kursor dla samej etykiety
			_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(_g_def._label) + 2) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));
		}

		_gfx_dev->printf(" %s ", _g_def._label);
	}

	if (_g_def._val_txt) _gfx_dev->printf(" %s ", buf);
}

void HGauge::fill_gauge(uint16_t colour, uint8_t gval)
{
	_gfx_dev->fillRect(_g_def._x + 2, _g_def._y + 2, gval, _g_def._h - 4, colour);
}

void HGauge::fill_gauge(enum g_style style, uint8_t gval)
{
	uint16_t colour;
	uint8_t start_val = 0, stop_val = 0;

	switch (style)
	{
		case G_M_B2R:
			start_val = 0;
			stop_val = 127;
			break;

		case G_M_G2R:
			start_val = 63;
			stop_val = 127;
			break;

		case G_M_R2G:
			start_val = 127;
			stop_val = 63;
			break;

		default:
			break;
	}

	for (uint8_t line = 0; line < gval; line++)
	{
		colour = rainbow(map(line, 0, _g_def._w, start_val, stop_val));
		_gfx_dev->drawFastVLine(_g_def._x + 2 + line, _g_def._y + 2, _g_def._h - 4, colour);
	}
}

VGauge::VGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c,
               uint16_t fg_c, bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val)
{
	_gfx_dev = gfx_dev;
	_g_def._x = x;
	_g_def._y = y;
	_g_def._w = w;
	_g_def._h = h;
	_g_def._back_c = back_c;
	_g_def._frame_c = frame_c;
	_g_def._fg_c = fg_c;
	_g_def._val_txt = val_txt;
	_g_def._label = (char *) label;
	_g_def._txt_size = txt_size;
	_g_def._style = gstyle;
	_g_def._min_val = min_val;
	_g_def._max_val = max_val;
	_g_def._value = min_val;
}

void VGauge::show(void)
{
	// Ramka
	_gfx_dev->drawRect(_g_def._x, _g_def._y, _g_def._w, _g_def._h, _g_def._frame_c);
	// Zawartosc wskaznika (poczatkowa)
	update(_g_def._value);
}

void VGauge::update(int16_t value)
{
	// Ograniczenie wartosci i mapowanie na wysokosc wskaznika
	uint8_t gval = map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, _g_def._h - 4, 0);
	bool newval_lower;

	// Jezeli nowa wwartosc wskaznika mniejsza od wyswietlanej, to zamaz wskazanie (tylko powyzej poprzedniej wartosci)
	if (value < _g_def._value)
	{
		newval_lower = true;
		_gfx_dev->fillRect(_g_def._x + 1, _g_def._y + 1, _g_def._w - 2, gval, _g_def._back_c);
	}
	else newval_lower = false;

	_g_def._value = value;
	uint8_t segs = _g_def._h / SEGMENT_S;										// Ilosc "segmentow" we wskazniku
	uint8_t snum = map(value, _g_def._min_val, _g_def._max_val, 0, segs);		// Ilosc segmentow odpowiadajaca wartosci
	char buf[8];

	if (_g_def._val_txt) itoa(value, buf, 10);									// Jezeli flaga val_txt jest ustawiona, to opisem jest wartosc

	uint16_t colour;

	switch (_g_def._style)														// W zaleznosci od stylu wskaznika rysuj zawartosc
	{
		case G_PLAIN:
			// Wskaznik (0, 0, 10, 20)
			//   0123456789
			// 0 ----------
			// 1 |        |
			// 2 |--------|
			// 3 |--------|
			// 4 |--------|
			// 5 |--------|
			// 6 |--------|
			// 7 |--------|
			// 8 |--------|
			// 9 |--------|
			// 0 |--------|
			// 1 |--------|
			// 2 |--------|
			// 3 |--------|
			// 4 |--------|
			// 5 |--------|
			// 6 |--------|
			// 7 |--------|
			// 8 |        |
			// 9 ----------
			colour = _g_def._fg_c;

			if (!newval_lower) fill_gauge(colour, gval);						// Jezeli nowa wartosc wieksza od poprzedniej rysuj zawartosc

			break;

		case G_SEGMENTS:

			// Wskaznik (0, 0, 10, 20)
			//   0123456789
			// 0 ----------
			// 1 |        |
			// 2 |--------|
			// 3 |--------|
			// 4 |--------|
			// 5 |        |
			// 6 |        |
			// 7 |        |
			// 8 |        |
			// 9 |--------|
			// 0 |--------|
			// 1 |--------|
			// 2 |--------|
			// 3 |--------|
			// 4 |--------|
			// 5 |--------|
			// 6 |--------|
			// 7 |--------|
			// 8 |--------|
			// 9 ----------
			if (!newval_lower)													// Jezeli nowa wartosc wieksza od poprzedniej rysuj zawartosc
			{
				for (uint8_t seg = 0; seg < snum; seg++) _gfx_dev->fillRect(_g_def._x + 2, _g_def._y + _g_def._h - ((seg  + 1) * SEGMENT_S) + 2, _g_def._w - 4, SEGMENT_W, _g_def._fg_c);
			}

			break;

		case G_B2R:
			colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 0, 127)); // Spectrum B -> R
			fill_gauge(colour, gval);											// Rysuj zawartosc
			break;

		case G_G2R:
			colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 63, 127)); // Spectrum G -> R
			fill_gauge(colour, gval);											// Rysuj zawartosc
			break;

		case G_R2G:
			colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 127, 63)); // Spectrum R -> G
			fill_gauge(colour, gval);											// Rysuj zawartosc
			break;

		case G_M_B2R:
		case G_M_R2G:
		case G_M_G2R:
			fill_gauge(_g_def._style, gval);
			break;

		default:
			break;
	}

	_gfx_dev->setTextFont(1);
	_gfx_dev->setTextColor(_g_def._frame_c, _g_def._back_c);
	_gfx_dev->setTextSize(_g_def._txt_size);
	// Kursor dla samej wartosci
	_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(buf) + 2) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));

	if (_g_def._val_txt) _gfx_dev->printf(" %s ", buf);

	if (strlen(_g_def._label))
	{
		if (_g_def._val_txt)
		{
			// Kursor dla etykiety z wartoscia
			_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(_g_def._label) + 2) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 4) - (4 * _g_def._txt_size));
		}
		else
		{
			// Kursor dla samej etykiety
			_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(_g_def._label) + 2) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));
		}

		_gfx_dev->printf(" %s ", _g_def._label);
	}
}

void VGauge::fill_gauge(uint16_t colour, uint8_t gval)
{
	_gfx_dev->fillRect(_g_def._x + 2, _g_def._y + gval + 2, _g_def._w - 4, _g_def._h - gval - 4, colour);
}

void VGauge::fill_gauge(enum g_style style, uint8_t gval)
{
	uint16_t colour;
	uint8_t start_val = 0, stop_val = 0;

	switch (style)
	{
		case G_M_B2R:
			start_val = 0;
			stop_val = 127;
			break;

		case G_M_G2R:
			start_val = 63;
			stop_val = 127;
			break;

		case G_M_R2G:
			start_val = 127;
			stop_val = 63;
			break;

		default:
			break;
	}

	for (uint8_t line = _g_def._h; line > (gval + 2); line--)
	{
		colour = rainbow(map(line, _g_def._h, 0, start_val, stop_val));
		_gfx_dev->drawFastHLine(_g_def._x + 2, _g_def._y + line - 2, _g_def._w - 4, colour);
	}
}

ARCGauge::ARCGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c,
                   uint16_t fg_c, bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val)
{
	_gfx_dev = gfx_dev;
	_g_def._x = x;
	_g_def._y = y;
	_g_def._w = w;
	_g_def._h = h;
	_g_def._back_c = back_c;
	_g_def._frame_c = frame_c;
	_g_def._fg_c = fg_c;
	_g_def._val_txt = val_txt;
	_g_def._label = (char *) label;
	_g_def._txt_size = txt_size;
	_g_def._style = gstyle;
	_g_def._min_val = min_val;
	_g_def._max_val = max_val;
	_g_def._value = min_val;
}

void ARCGauge::show(void)
{
	// Ramka
	//_gfx_dev->drawCircle(_g_def._x + (_g_def._w / 2), _g_def._y + (_g_def._w / 2), _g_def._w / 2, _g_def._frame_c);
	_gfx_dev->drawRect(_g_def._x, _g_def._y, _g_def._w, _g_def._h, _g_def._frame_c);
	// Zawartosc wskaznika (poczatkowa)
	update(_g_def._value);
}

void ARCGauge::update(int16_t value)
{
	uint8_t r = _g_def._w / 2;													// Promien
	uint8_t x = _g_def._x + r;													// Centrum wskaznika
	uint8_t y = _g_def._y + r;
	int8_t w = r / 4;															// Grubosc zewnetrznego okregu to 1/4 promienia
	// Ograniczenie wartosci i mapowanie na kat wskaznika
	int16_t gval = map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, -ARCANGLE, ARCANGLE);
	uint16_t colour = 0;
	uint8_t segs = 1;															// Dla wartosci > 1 wskaznik typu 'segmenty'
	_g_def._value = value;

	if (_g_def._style == G_SEGMENTS) segs = 2;

	for (int16_t i = -ARCANGLE; i < ARCANGLE; i += (segs * ARCSEGS_D))			// Rysuj segmenty co zdefiniowany odstep
	{
		switch (_g_def._style)
		{
			case G_PLAIN:
			case G_SEGMENTS:
				colour = _g_def._fg_c;
				break;

			case G_B2R:
				colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 0, 127)); // Spectrum B -> R
				break;

			case G_G2R:
				colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 63, 127)); // Spectrum G -> R
				break;

			case G_R2G:
				colour = rainbow(map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 127, 63)); // Spectrum R -> G
				break;

			case G_M_B2R:
			case G_M_R2G:
			case G_M_G2R:
				break;

			default:
				break;
		}

		float sx = cos((i - 90) * 0.0174532925);								// Oblicz koordynaty poczatku segmentu
		float sy = sin((i - 90) * 0.0174532925);
		uint16_t x0 = sx * (r - w) + x;
		uint16_t y0 = sy * (r - w) + y;
		uint16_t x1 = sx * r + x;
		uint16_t y1 = sy * r + y;
		float sx2 = cos((i + ARCSEGS_W - 90) * 0.0174532925);					// Oblicz koordynaty konca segmentu
		float sy2 = sin((i + ARCSEGS_W - 90) * 0.0174532925);
		int16_t x2 = sx2 * (r - w) + x;
		int16_t y2 = sy2 * (r - w) + y;
		int16_t x3 = sx2 * r + x;
		int16_t y3 = sy2 * r + y;

		if (i < gval)															// Wypelniaj dwoma trojkatami aktywny obszar
		{
			_gfx_dev->fillTriangle(x0, y0, x1, y1, x2, y2, colour);
			_gfx_dev->fillTriangle(x1, y1, x2, y2, x3, y3, colour);
		}
		else																	// Nieaktywny obszar wypelniony na szaro
		{
			_gfx_dev->fillTriangle(x0, y0, x1, y1, x2, y2, TFT_LIGHTGREY);
			_gfx_dev->fillTriangle(x1, y1, x2, y2, x3, y3, TFT_LIGHTGREY);
		}
	}

	char buf[8];																// Konwersja wartosci na string

	if (_g_def._val_txt) itoa(value, buf, 10);									// Jezeli flaga val_txt jest ustawiona, to opisem jest wartosc

	_gfx_dev->setTextFont(1);
	_gfx_dev->setTextColor(_g_def._frame_c, _g_def._back_c);
	_gfx_dev->setTextSize(_g_def._txt_size);
	// Kursor dla samej wartosci
	_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(buf) + 2) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));

	if (strlen(_g_def._label))
	{
		if (_g_def._val_txt)
		{
			// Kursor dla etykieta z wartoscia
			_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(_g_def._label) + strlen(buf) + 3) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));
		}
		else
		{
			// Kursor dla samej etykiety
			_gfx_dev->setCursor(_g_def._x + (_g_def._w / 2) - ((strlen(_g_def._label) + 1) * 3 * _g_def._txt_size), _g_def._y + (_g_def._h / 2) - (4 * _g_def._txt_size));
		}

		_gfx_dev->printf(" %s", _g_def._label);
	}

	if (_g_def._val_txt) _gfx_dev->printf(" %s ", buf);
}

ChartGauge::ChartGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c, uint16_t fg_c,
                       bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val)
{
	_gfx_dev = gfx_dev;
	_g_def._x = x;
	_g_def._y = y;
	_g_def._w = w;
	_g_def._h = h;
	_g_def._back_c = back_c;
	_g_def._frame_c = frame_c;
	_g_def._fg_c = fg_c;
	_g_def._val_txt = val_txt;
	_g_def._label = (char *) label;
	_g_def._txt_size = txt_size;
	_g_def._style = gstyle;
	_g_def._min_val = min_val;
	_g_def._max_val = max_val;
	_g_def._value = min_val;
	_sample = 0;																// Ilosc sampli
	_full_chart = false;														// Flaga pelnego wykresu
}

ChartGauge::~ChartGauge(void)
{
	free(_chart_data);
}

void ChartGauge::show(void)
{
	// Ramka (pusta)
	_gfx_dev->drawRect(_g_def._x, _g_def._y, _g_def._w, _g_def._h, _g_def._frame_c);
	// Alokacja pamieci dla danych - bufor
	_chart_data = (uint8_t *)calloc(_g_def._w - 2, sizeof(uint8_t));			// Szerokosc wskaznika minus ramka
}

void ChartGauge::update(int16_t value)
{
	// Czyszczenie zawartosci
	_gfx_dev->fillRect(_g_def._x + 1, _g_def._y + 1, _g_def._w - 2, _g_def._h - 2, _g_def._back_c);
	// Srodkowa os pozioma
	_gfx_dev->drawFastHLine(_g_def._x + 2, _g_def._y + (_g_def._h / 2), _g_def._w - 4, _g_def._frame_c);
	// Ograniczenie wartosci i mapowanie na wysokosc wskaznika
	uint8_t gval = map(constrain(value, _g_def._min_val, _g_def._max_val), _g_def._min_val, _g_def._max_val, 0, _g_def._h - 4);
	uint8_t *sample_adr;
	// Bufor cykliczny
	sample_adr = _chart_data + (_sample % (_g_def._w - 2));						// Adres w buforze modulo szerokosc (bez ramki)

	// Jezeli ilosc probek rowna szerokosci (bez ramki), ustaw flage pelnego wykresu
	if (_sample > (_g_def._w - 3)) _full_chart = true;

	*sample_adr = gval;															// Zapisz do bufora
	uint8_t *cur_data;
	uint16_t data_ptr = 0;

	// Opis
	char buf[8];

	if (_g_def._val_txt) itoa(value, buf, 10);									// Jezeli flaga val_txt jest ustawiona, to opisem jest wartosc

	_gfx_dev->setTextFont(1);
	_gfx_dev->setTextColor(_g_def._frame_c, _g_def._back_c);
	_gfx_dev->setTextSize(_g_def._txt_size);
	_gfx_dev->setCursor(_g_def._x + 10, _g_def._y + (_g_def._h / 4) - (4 * _g_def._txt_size));

	if (strlen(_g_def._label)) _gfx_dev->printf("%s", _g_def._label);			// Jezeli jest niezerowa etykieta

	if (_g_def._val_txt) _gfx_dev->printf("%s", buf);							// Jezeli flaga wartosci jako etykiety

	// Jezeli ilosc danych zapelnia wykres
	if (_full_chart)
	{
		for (uint16_t i = 0; i < (_g_def._w - 2); i++)
		{
			cur_data = sample_adr + i;

			if (cur_data >= _chart_data + (_g_def._w - 2))						// Jezeli wskaznik przekracza koniec bufora cyklicznego
			{
				cur_data = _chart_data + data_ptr;								// Pobieraj dane z powrotem od jego poczatku
				data_ptr++;														// Nastepna dana do wykresu
			}

			switch (_g_def._style)
			{
				case C_DOTS:
					_gfx_dev->drawPixel(_g_def._x + 1 + i, _g_def._y + _g_def._h - 2 - *cur_data, _g_def._fg_c);
					break;

				case  C_3DOTS:
					_gfx_dev->drawPixel(_g_def._x + 1 + i, _g_def._y + _g_def._h - 3 - *cur_data, _g_def._fg_c);
					_gfx_dev->drawPixel(_g_def._x + 1 + i, _g_def._y + _g_def._h - *cur_data - 2, _g_def._fg_c);
					_gfx_dev->drawPixel(_g_def._x + 1 + i, _g_def._y + _g_def._h - *cur_data  - 4, _g_def._fg_c);
					break;

				case C_LINES:
					_gfx_dev->drawFastVLine(_g_def._x + 1 + i, _g_def._y + _g_def._h - 2 - *cur_data, *cur_data, _g_def._fg_c);
					break;

				default:
					break;
			}
		}
	}
	// Jezeli mniej danych niz szerokosc wykresu
	else
	{
		for (uint16_t i = 0; i < (_sample  + 1); i++)
		{
			cur_data = sample_adr - i;

			switch (_g_def._style)
			{
				case C_DOTS:
					_gfx_dev->drawPixel(_g_def._x + _g_def._w - 2 - i, _g_def._y + _g_def._h - 3 - *cur_data, _g_def._fg_c);
					break;

				case  C_3DOTS:
					_gfx_dev->drawPixel(_g_def._x + _g_def._w - 2 - i, _g_def._y + _g_def._h - 3 - *cur_data, _g_def._fg_c);
					_gfx_dev->drawPixel(_g_def._x + _g_def._w - 2 - i, _g_def._y + _g_def._h - *cur_data - 2, _g_def._fg_c);
					_gfx_dev->drawPixel(_g_def._x + _g_def._w - 2 - i, _g_def._y + _g_def._h - *cur_data - 4, _g_def._fg_c);
					break;

				case C_LINES:
					_gfx_dev->drawFastVLine(_g_def._x + _g_def._w - 2 - i, _g_def._y + _g_def._h - 2 - *cur_data , *cur_data, _g_def._fg_c);
					break;

				default:
					break;
			}
		}
	}

	_sample++;																	// Nastepny adres cykliczny
}

void ChartGauge::end(void)
{
	free(_chart_data);
	_sample = 0;
	_full_chart = false;
}

Chart2Gauge::Chart2Gauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c, uint16_t fg_c1, uint16_t fg_c2,
                         bool val_txt, const char *label1, const char *label2, uint8_t txt_size, int16_t min_val1, int16_t max_val1, int16_t min_val2, int16_t max_val2)
{
	_gfx_dev = gfx_dev;
	_g_def._x = x;
	_g_def._y = y;
	_g_def._w = w;
	_g_def._h = h;
	_g_def._back_c = back_c;
	_g_def._frame_c = frame_c;
	_g_def._fg_c1 = fg_c1;
	_g_def._fg_c2 = fg_c2;
	_g_def._val_txt = val_txt;
	_g_def._label1 = (char *) label1;
	_g_def._label2 = (char *) label2;
	_g_def._txt_size = txt_size;
	_g_def._min_val1 = min_val1;
	_g_def._max_val1 = max_val1;
	_g_def._min_val2 = min_val2;
	_g_def._max_val2 = max_val2;
	_sample = 0;																// Ilosc sampli
	_full_chart = false;														// Flaga pelnego wykresu
}

Chart2Gauge::~Chart2Gauge(void)
{
	free(_chart_data);
}

void Chart2Gauge::show(void)
{
	// Ramka (pusta)
	_gfx_dev->drawRect(_g_def._x, _g_def._y, _g_def._w, _g_def._h, _g_def._frame_c);
	// Alokacja pamieci dla danych - bufor
	_chart_data = (int16_t *)calloc(_g_def._w - 2, sizeof(int16_t));			// Szerokosc wskaznika minus ramka (dane dwubajtowe)
}

void Chart2Gauge::update(int16_t val1, int16_t val2)
{
	// Czyszczenie zawartosci
	_gfx_dev->fillRect(_g_def._x + 1, _g_def._y + 1, _g_def._w - 2, _g_def._h - 2, _g_def._back_c);
	// Srodkowa os pozioma
	_gfx_dev->drawFastHLine(_g_def._x + 2, _g_def._y + (_g_def._h / 2), _g_def._w - 4, _g_def._frame_c);
	// Ograniczenie wartosci i mapowanie na wysokosc wskaznika
	uint8_t gval1 = map(constrain(val1, _g_def._min_val1, _g_def._max_val1), _g_def._min_val1, _g_def._max_val1, 0, _g_def._h - 4);
	uint8_t gval2 = map(constrain(val2, _g_def._min_val2, _g_def._max_val2), _g_def._min_val2, _g_def._max_val2, 0, _g_def._h - 4);
	int16_t *sample_adr;
	// Bufor cykliczny
	sample_adr = _chart_data + (_sample % (_g_def._w - 2));						// Adres w buforze modulo szerokosc (bez ramki)

	// Jezeli ilosc probek rowna szerokosci (bez ramki), ustaw flage pelnego wykresu
	if (_sample > (_g_def._w - 3)) _full_chart = true;

	*sample_adr = (gval1 << 8) + gval2;											// Zapisz do bufora (MSB | LSB)
	int16_t *cur_data;
	uint16_t data_ptr = 0;

	// Opis
	char buf1[8], buf2[8];

	if (_g_def._val_txt)
	{
		itoa(val1, buf1, 10);
		itoa(val2, buf2, 10);
	}

	_gfx_dev->setTextFont(1);
	_gfx_dev->setTextSize(_g_def._txt_size);
	// Etykiety
	_gfx_dev->setTextColor(_g_def._fg_c1, _g_def._back_c);
	_gfx_dev->setCursor(_g_def._x + 10, _g_def._y + (_g_def._h / 4) - (4 * _g_def._txt_size));

	if (strlen(_g_def._label1)) _gfx_dev->printf("%s", _g_def._label1);

	// Jezeli flaga wartosci jako etykiety
	if (_g_def._val_txt) _gfx_dev->printf("%s", buf1);

	_gfx_dev->setTextColor(_g_def._fg_c2, _g_def._back_c);
	_gfx_dev->setCursor(_g_def._x + 10, _g_def._y + ((_g_def._h / 4) * 3) - (4 * _g_def._txt_size));

	if (strlen(_g_def._label2))	_gfx_dev->printf("%s", _g_def._label2);

	// Jezeli flaga wartosci jako etykiety
	if (_g_def._val_txt) _gfx_dev->printf("%s", buf2);

	// Jezeli ilosc danych zapelnia wykres
	if (_full_chart)
	{
		for (uint16_t i = 0; i < (_g_def._w - 2); i++)
		{
			cur_data = sample_adr + i;

			if (cur_data >= _chart_data + (_g_def._w - 2))						// Jezeli wskaznik przekracza koniec bufora cyklicznego
			{
				cur_data = _chart_data + data_ptr;								// Pobieraj dane z powrotem od jego poczatku
				data_ptr++;														// Nastepna dana do wykresu
			}

			// seria1 (MSB)
			_gfx_dev->drawPixel(_g_def._x + 1 + i, _g_def._y + _g_def._h - 2 - ((*cur_data) >> 8), _g_def._fg_c1);
			// seria2 (LSB)
			_gfx_dev->drawPixel(_g_def._x + 1 + i, _g_def._y + _g_def._h - 2 - ((*cur_data) & 0xFF), _g_def._fg_c2);
		}
	}
	// Jezeli mniej danych niz szerokosc wykresu
	else
	{
		for (uint16_t i = 0; i < (_sample  + 1); i++)
		{
			cur_data = sample_adr - i;
			// seria1 (MSB)
			_gfx_dev->drawPixel(_g_def._x + _g_def._w - 2 - i, _g_def._y + _g_def._h - 3 - ((*cur_data) >> 8), _g_def._fg_c1);
			// seria2 (LSB)
			_gfx_dev->drawPixel(_g_def._x + _g_def._w - 2 - i, _g_def._y + _g_def._h - 3 - ((*cur_data) & 0xFF), _g_def._fg_c2);
		}
	}

	_sample++;																	// Nastepny adres cykliczny
}

void Chart2Gauge::end(void)
{
	free(_chart_data);
	_sample = 0;
	_full_chart = false;
}