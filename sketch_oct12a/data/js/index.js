const refreshInterval = 2000;
var funcInterval = null;
function getLightData(room, area) {
	var xhttp = new XMLHttpRequest();
	xhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			var data = JSON.parse(this.responseText);
			if (room == 'Shower' || room == 'Table')		
				btnSwDesigner((data.lightLvl > 0) ? 'ON' : 'OFF');
			else {
				$('#lightLvl').val(data.lightLvl);
				$('label[for="lightLvl"]').text(lightLvlFormatter(data.lightLvl));
				$('#minLightLvl').val(data.minLightLvl);
				$('label[for="minLightLvl"]').text(minLightLvlFormatter(data.minLightLvl));
			}
			switch (area) {
				case 'Group':
					if (room == 'Toilet') {
						$('#fadeDelay').hide();
						$('label[for="fadeDelay"]').hide();
					} else {
						$('#fadeDelay').show();
						$('label[for="fadeDelay"]').show();
					}
					$('#fadeRate').show();
					$('label[for="fadeRate"]').show();
					if (room == 'Stairs') {
						$('#msllt').show();
						$('#msenState').text(msActFormatter(data.msenState));
						$('#infoMsen').show();
						$('label[for="msllt"]').show();
						$('#msllt').val(data.msllt);
						$('label[for="msllt"]').text(mslltFormatter(data.msllt));
					}
					else {
						$('#msllt').hide();
						$('label[for="msllt"]').hide();
					}
					$('#infoLight').hide();
					$('#fadeRate').val(data.fadeRate);
					$('label[for="fadeRate"]').text(fadeRateFormatter(data.fadeRate));
					$('#fadeDelay').val(data.fadeDelay);
					$('label[for="fadeDelay"]').text(fadeDelayFormatter(data.fadeDelay));
					break;
				case 'Lamp A':
				case 'Lamp B':
				case 'Lamp C':
					$('#fadeRate').hide();
					$('label[for="fadeRate"]').hide();
					$('#fadeDelay').hide();
					$('label[for="fadeDelay"]').hide();
					$('#msllt').hide();
					$('label[for="msllt"]').hide();
					$('#lampOT').text('On time: ' + secondsToHMS(data.lampOT));
					$('#lampCycles').text('Switching cycles: ' + data.lampCycles);
					$('#infoLight').show();
					break;
			}
			if (room == 'Shower' || room == 'Table')
				$('#btnSwitch').show();
			else
				$('#slidersForm').show();
			$('div[class="alert alert-warning"]').hide();
		} else if (this.readyState == 4 && this.status == 503) {
			$('div[class="alert alert-warning"]').show();
			$('div[class="alert alert-warning"]').text(this.responseText);
		}
		if (funcInterval != null)
			clearTimeout(funcInterval);
		funcInterval = setTimeout(getLightData, refreshInterval, room, area);
	};
	xhttp.open('POST', 'getl', true);
	xhttp.setRequestHeader('Content-type', 'application/json');
	xhttp.send(JSON.stringify({ 'room': room, 'area': area }));
}
function setLightData(room, area, dtype, dval, ltext) {	
	if (funcInterval != null)
		clearTimeout(funcInterval);
	var selector = 'label[for="' + dtype + '"]';
	var xhttp = new XMLHttpRequest();
	xhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			if (room == 'Shower' || room == 'Table')
				btnSwDesigner(ltext);
			else
				$(selector).text(ltext);
			$('div[class="alert alert-warning"]').hide();
		} else if (this.readyState == 4 && this.status == 503) {
			$('div[class="alert alert-warning"]').show();
			$('div[class="alert alert-warning"]').text(this.responseText);
		}
		funcInterval = setTimeout(getLightData, refreshInterval, room, area);
	};
	xhttp.open('POST', 'setl', true);
	xhttp.setRequestHeader('Content-type', 'application/json');
	xhttp.send(JSON.stringify({ 'room': room, 'area': area, 'dtype': dtype, 'dval': dval }));
}
function getVentData() {
	var xhttp = new XMLHttpRequest();
	xhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			var data = JSON.parse(this.responseText);	
			$('#ventFRPM').text('Front RPM: ' + data.ventFRPM);
			$('#ventRRPM').text('Rear RPM: ' + data.ventRRPM);
			$('#ventI').text('Current: ' + (data.ventI / 1000.0) + 'A');
			$('#ventT').html('Temperature: ' + (data.ventT / 10.0) + '&deg;C');
			$('#ventRH').text('RH: ' + (data.ventRH / 10.0) + '%');
			$('#minRH').val(data.minRH);
			$('label[for="minRH"]').text(minRHFormatter(data.minRH));
			$('#fanLvl').val(data.fanLvl);
			$('label[for="fanLvl"]').text(fanLvlFormatter(data.fanLvl));
			$('#slidersForm2').show();
			$('div[class="alert alert-warning"]').hide();
		} else if (this.readyState == 4 && this.status == 503) {
			$('div[class="alert alert-warning"]').show();
			$('div[class="alert alert-warning"]').text(this.responseText);
		}
		if (funcInterval != null)
			clearTimeout(funcInterval);
		funcInterval = setTimeout(getVentData, refreshInterval);
	};
	xhttp.open('GET', 'getv', true);		
	xhttp.send();
}
function setVentData(dtype, dval, ltext) {
	if (funcInterval != null)
		clearTimeout(funcInterval);
	var selector = 'label[for="' + dtype + '"]';
	var xhttp = new XMLHttpRequest();
	xhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			$(selector).text(ltext);
			$('div[class="alert alert-warning"]').hide();
		} else if (this.readyState == 4 && this.status == 503) {
			$('div[class="alert alert-warning"]').show();
			$('div[class="alert alert-warning"]').text(this.responseText);
		}
		funcInterval = setTimeout(getVentData, refreshInterval);
	};
	xhttp.open('POST', 'setv', true);
	xhttp.setRequestHeader('Content-type', 'application/json');
	xhttp.send(JSON.stringify({ 'dtype': dtype, 'dval': dval }));
}
function getStats() {
	var xhttp = new XMLHttpRequest();
	xhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			var data = JSON.parse(this.responseText);	
			$('#contr1OT').text('Controller 1: ' + secondsToHMS(data.contr1OT));
			$('#contr2OT').text('Controller 2: ' + secondsToHMS(data.contr2OT));
			$('div[class="alert alert-warning"]').hide();
		} else if (this.readyState == 4 && this.status == 503) {
			$('div[class="alert alert-warning"]').show();
			$('div[class="alert alert-warning"]').text(this.responseText);
		}
		if (funcInterval != null)
			clearTimeout(funcInterval);
		funcInterval = setTimeout(getStats, refreshInterval);
	};
	xhttp.open('GET', 'getc', true);		
	xhttp.send();	
}
function areaSelect(area) {
	$('#slidersForm').hide();
	$('#deletionForm').hide();
	$('.alert').hide();
	$('#menuLamps').text(area);
	$('#infoMsen').hide();
	if (funcInterval != null)
		clearTimeout(funcInterval);
	getLightData($('#menuRooms').text(), area);
}
function roomSelect(room) {
	$('#slidersForm').hide();
	$('#btnSwitch').hide();
	switch (room) {
		case 'Kitchen':
		case 'Room A':
		case 'Room B':
		case 'Room C':
			$('#m2la').show();
			$('#m2lb').show();
			$('#m2lc').show();
			break;
		case 'Table':
		case 'Shower':
		case 'Toilet':
			$('#m2la').show();
			$('#m2lb').hide();
			$('#m2lc').hide();
			break;
		case 'Stairs':
			$('#m2la').show();
			$('#m2lb').show();
			$('#m2lc').hide();
			break;
	}
	$('#menuRooms').text(room);
	areaSelect('Group');
}
function lightLvlFormatter(value) {
	return 'Light level: ' + (value < 1 ? 'Off' : (value + '%'));
}
function minLightLvlFormatter(value) {
	return 'Minimum light level: ' + value + '%';
}
function fadeRateFormatter(value) {
	return 'Fade rate: ' + (3 * value / 8.0) + '%/s';
}
function fadeDelayFormatter(value) {
	return 'Fade delay: ' + (value / 32.0) + 's';
}
function mslltFormatter(value) {
	return 'Motion sensor low level time: ' + (value < 6 ? 'Disabled' : (value + 's'));
}
function fanLvlFormatter(value) {
	return 'Fan level: ' + (value > 99 ? 'Auto' : value + '%');
}
function minRHFormatter(value) {
	return 'Minimum RH: ' + value + '%';
}
function msActFormatter(value) {
	var result = 'Motion sensor activity: ';
	if (value < 20)
		result += 'N/C';
    else if (value < 99)
		result += 'off';
    else if (value < 121)
		result += 'upper';
    else if (value < 142)
		result += 'lower';
    else
		result += 'both';
	return result;
}
function btnSwDesigner(ltext) {
	$('#btnSwitch').text(ltext);
	if (ltext == 'OFF') {
		$('#btnSwitch').toggleClass('btn-primary', false);
		$('#btnSwitch').toggleClass('btn-secondary', true);
	} else if (ltext == 'ON') {
		$('#btnSwitch').toggleClass('btn-secondary', false);
		$('#btnSwitch').toggleClass('btn-primary', true);
	}
}
function secondsToHMS(value) {
	var result = '';
	const days = Math.floor(value / 86400);
	if (days > 0) {
		result += days + ' day';
		if (days > 1)
			result += 's';
	}
	const hours = Math.floor(value / 3600) % 24;
	if (hours > 0) {
		result += ' ' + hours + ' hour';
		if (hours > 1)
			result += 's';
	}
	const minutes = Math.floor(value / 60) % 60;
	if (minutes > 0) {
		result += ' ' + minutes + ' minute';
		if (minutes > 1)
			result += 's';
	}
	return result;
}
$(document).ready(function(){
	$('.nav-tabs a').click(function(){
		$(this).tab('show');
	});
	$('.nav-tabs a[href="#tab_light"]').on('shown.bs.tab', function(){
		if ($('#menuRooms').text() == 'Rooms')
			roomSelect('Kitchen');
		else
			areaSelect($('#menuLamps').text());
	});
	$('.nav-tabs a[href="#tab_vent"]').on('shown.bs.tab', function(){
		$('#slidersForm2').hide();
		getVentData();
	});
	$('.nav-tabs a[href="#tab_configuration"]').on('shown.bs.tab', function(){
		getStats();
	});
	$('.custom-range').input(function() {
		if (funcInterval != null)
			clearTimeout(funcInterval);
	});
	$('#lightLvl').change(function() {
		var value = $('#lightLvl').val();
		var ltext = lightLvlFormatter(value);
		setLightData($('#menuRooms').text(), $('#menuLamps').text(), 'lightLvl', Math.floor(value * 2.55), ltext);
	});
	$('#minLightLvl').change(function() {
		var value = $('#minLightLvl').val();
		var ltext = minLightLvlFormatter(value);
		setLightData($('#menuRooms').text(), $('#menuLamps').text(), 'minLightLvl', Math.floor(value * 2.55), ltext);
	});
	$('#fadeRate').change(function() {
		var value = $('#fadeRate').val();
		var ltext = fadeRateFormatter(value);
		setLightData($('#menuRooms').text(), $('#menuLamps').text(), 'fadeRate', value, ltext);
	});
	$('#fadeDelay').change(function() {
		var value = $('#fadeDelay').val();
		var ltext = fadeDelayFormatter(value);
		setLightData($('#menuRooms').text(), $('#menuLamps').text(), 'fadeDelay', value, ltext);
	});
	$('#msllt').change(function() {
		var value = $('#msllt').val();
		var ltext = mslltFormatter(value);
		setLightData($('#menuRooms').text(), $('#menuLamps').text(), 'msllt', value, ltext);
	});
	$('#fanLvl').change(function() {
		var value = $('#fanLvl').val();
		var ltext = fanLvlFormatter(value);
		setVentData('fanLvl', value, ltext);
	});
	$('#minRH').change(function() {
		var value = $('#minRH').val();
		var ltext = minRHFormatter(value);
		setVentData('minRH', value, ltext);
	});
	$('#lampCycles').dblclick(function() {
		if ($('#menuLamps').text() != 'Group')
			$('#deletionForm').show();
	});
	$('#delTextButton').click(function() {
		if (area == 'Group') return;
		var room = $('#menuRooms').text();
		var area = $('#menuLamps').text();
		var code = $('#delText').val();
		$('#deletionForm').hide();
		var xhttp = new XMLHttpRequest();
		xhttp.onreadystatechange = function() {
			if (this.readyState == 4) {
				if (this.status == 200) {
					$('div[class="alert alert-success"]').show();
					$('div[class="alert alert-warning"]').hide();
				} else if (this.status == 503) {
					$('div[class="alert alert-warning"]').show();
					$('div[class="alert alert-warning"]').text(this.responseText);
				}
			};
		};
		xhttp.open('POST', 'resh', true);
		xhttp.setRequestHeader('Content-type', 'application/json');
		xhttp.send(JSON.stringify({ 'room': room, 'area': area, 'code': code }));
	});
	$('#btnSwitch').click(function() {
		if ($('#btnSwitch').text() == 'ON') {
			setLightData($('#menuRooms').text(), $('#menuLamps').text(), 'lightLvl', 0, 'OFF');
		} else if ($('#btnSwitch').text() == 'OFF') {
			setLightData($('#menuRooms').text(), $('#menuLamps').text(), 'lightLvl', 25, 'ON');
		}
	});
});
