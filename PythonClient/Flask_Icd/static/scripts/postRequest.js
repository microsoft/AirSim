
$("#mybutton").click(function (event) {
    $.ajax({
        type: 'POST',
        url: "{{url_for('test')}}",
        contentType: "application/json",
        data: JSON.stringify("yigal"),
        dataType: 'json'
    });
});