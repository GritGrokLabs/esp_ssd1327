# SSD1327 Component

This is my first attempt at creating a component, so please be patient, kind and gentle! It took me ~32 hours to get everything working, so I thought I'd take a few more minutes and publish it online in hopes it might save someone else some time. That being said, I've only tested on a single display, so YMMV on how well this works for you!

## Installation

I had to add the following values to my sdkconfig.defaults to get everything to play nicely with one another...


```ini
# sdkconfig snippet
CONFIG_LV_COLOR_DEPTH_8=y
CONFIG_LV_USE_THEME_DEFAULT=n
CONFIG_LV_USE_THEME_BASIC=n
CONFIG_LV_USE_THEME_MONO=y

