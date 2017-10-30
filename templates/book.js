$(document).ready(function() {

  $('nav li a').on('click', function(){
      $('nav li a.active').removeClass('active');
      $(this).addClass('active');
  });
});
