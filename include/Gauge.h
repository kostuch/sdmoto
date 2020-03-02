#ifndef GAUGE_H_
#define GAUGE_H_

#include <TFT_eSPI.h>

#define SEGMENT_S	14															// Wielkosc segmentu w pikselach
#define SEGMENT_W	10															// Wypelnienie segmentu w pikselach
#define ARCSEGS_W	5															// Szerokosc katowa segmentu wskaznika ARC (5 stopni = 60 segmentow na 300 stopni)
#define ARCSEGS_D	5															// Odstep katowy pomiedzy segmentami wskaznika ARC (*2 = segmenty)
#define ARCANGLE	150															// Zakres katowy wskaznika ARC

enum g_style																	// Styl wskaznika
{
	G_PLAIN,																	// Jednolite wypelnienie kolorem
	G_SEGMENTS,																	// Segmenty jednego koloru
	G_B2R,																		// Jednolity kolor w zaleznosci od wartosci B->R
	G_G2R,																		// Jednolity kolor w zaleznosci od wartosci G->R
	G_R2G,																		// Jednolity kolor w zaleznosci od wartosci R->G
	G_M_B2R,																	// Multikolor w zaleznosci od wartosci B->R
	G_M_G2R,																	// Multikolor w zaleznosci od wartosci G->R
	G_M_R2G,																	// Multikolor w zaleznosci od wartosci R->G
	C_DOTS,																		// Wykres punktowy
	C_3DOTS,																	// Wykres trzypunktowy (pogrubiony)
	C_2SERIES,																	// Wykres z dwoma seriami
	C_LINES																		// Wykres kolumnowy
};

typedef struct
{
	uint16_t		_x;
	uint8_t			_y;
	uint16_t		_w;
	uint8_t			_h;
	uint16_t		_back_c;
	uint16_t		_frame_c;
	uint16_t		_fg_c;
	bool			_val_txt;
	char			*_label;
	uint8_t			_txt_size;
	enum g_style	_style;
	int16_t			_min_val;
	int16_t			_max_val;
	int16_t			_value;
} gauge_t;

typedef struct
{
	uint16_t		_x;
	uint8_t			_y;
	uint16_t		_w;
	uint8_t			_h;
	uint16_t		_back_c;
	uint16_t		_frame_c;
	uint16_t		_fg_c1;
	uint16_t		_fg_c2;
	bool			_val_txt;
	char			*_label1;
	char			*_label2;
	uint8_t			_txt_size;
	int16_t			_min_val1;
	int16_t			_max_val1;
	int16_t			_min_val2;
	int16_t			_max_val2;
} gauge2_t;

class Gauge
{
	friend class HGauge;
	friend class VGauge;
	friend class ARCGauge;
	friend class ChartGauge;
public:
	virtual void show(void);
	virtual void update(int16_t);
private:
	uint16_t rainbow(uint8_t value);
	gauge_t			_g_def;
	TFT_eSPI		*_gfx_dev;
};

class HGauge : public Gauge
{
public:
	HGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c, uint16_t fg_c,
	       bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val);
	void show(void);
	void update(int16_t value);
private:
	void fill_gauge(uint16_t colour, uint8_t gval);
	void fill_gauge(enum g_style style, uint8_t gval);
	TFT_eSPI		*_gfx_dev;
	gauge_t			_g_def;
};

class VGauge : public Gauge
{
public:
	VGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c, uint16_t fg,
	       bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val);
	void show(void);
	void update(int16_t value);
private:
	void fill_gauge(uint16_t colour, uint8_t gval);
	void fill_gauge(enum g_style style, uint8_t gval);
	TFT_eSPI		*_gfx_dev;
	gauge_t			_g_def;
};

class ARCGauge : public Gauge
{
public:
	ARCGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c, uint16_t fg_c,
	         bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val);
	void show(void);
	void update(int16_t value);
private:
	TFT_eSPI		*_gfx_dev;
	gauge_t			_g_def;
};

class ChartGauge : public Gauge
{
public:
	ChartGauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c, uint16_t fg_c,
	           bool val_txt, const char *label, uint8_t txt_size, enum g_style gstyle, int16_t min_val, int16_t max_val);
	~ChartGauge(void);
	void show(void);
	void update(int16_t value);
	void end(void);
private:
	TFT_eSPI		*_gfx_dev;
	gauge_t			_g_def;
	uint8_t			*_chart_data;
	uint16_t		_sample;
	bool			_full_chart;
};

class Chart2Gauge
{
public:
	Chart2Gauge(TFT_eSPI *gfx_dev, uint16_t x, uint8_t y, uint16_t w, uint8_t h, uint16_t back_c, uint16_t frame_c, uint16_t fg_c1, uint16_t fg_c2,
	            bool val_txt, const char *label1, const char *label2, uint8_t txt_size, int16_t min_val1, int16_t max_val1, int16_t min_val2, int16_t max_val2);
	~Chart2Gauge(void);
	void show(void);
	void update(int16_t val1, int16_t val2);
	void end(void);
private:
	TFT_eSPI		*_gfx_dev;
	gauge2_t		_g_def;
	int16_t			*_chart_data;
	uint16_t		_sample;
	bool			_full_chart;
};

#endif /* GAUGE_H_ */